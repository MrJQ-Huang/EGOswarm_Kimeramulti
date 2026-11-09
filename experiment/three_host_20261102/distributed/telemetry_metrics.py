#!/usr/bin/env python3
"""Hub-only bounded timestamp statistics; no raw messages or poses are saved.

Source age includes sender/receiver clock offset and every upstream queue. It
is not a measurement of one-way network latency. Actual-pair differences use
fixed-rate sampler ticks and the latest received source stamps, not synchronized
camera frames, callback-count weighting, interpolation or pose estimation.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import random
import signal
import threading
import time


class ScalarSummary:
    """Cumulative moments and a fixed-size uniform reservoir for percentiles."""
    def __init__(self,limit=8192):
        if type(limit) is not int or limit < 1: raise ValueError('Positive reservoir bound required')
        self.limit=limit;self.values=[];self.count=0;self.total=0.;self.minimum=None;self.maximum=None
        self.random=random.Random(0)

    def add(self,value):
        value=float(value)
        if not math.isfinite(value): raise ValueError('Nonfinite statistic')
        self.count+=1;self.total+=value
        self.minimum=value if self.minimum is None else min(self.minimum,value)
        self.maximum=value if self.maximum is None else max(self.maximum,value)
        if len(self.values)<self.limit:self.values.append(value)
        else:
            index=self.random.randrange(self.count)
            if index<self.limit:self.values[index]=value

    def report(self):
        if not self.count:return dict(samples=0,min=None,mean=None,p50=None,p95=None,max=None)
        values=sorted(self.values)
        def percentile(fraction):
            index=(len(values)-1)*fraction;lo=int(index);hi=min(lo+1,len(values)-1)
            return values[lo]+(values[hi]-values[lo])*(index-lo)
        return dict(samples=self.count,min=self.minimum,mean=self.total/self.count,
            p50=percentile(.5),p95=percentile(.95),max=self.maximum,
            percentile_method='exact' if self.count<=self.limit else 'bounded_uniform_reservoir',
            retained_scalars=len(values),reservoir_limit=self.limit)


class StreamStats:
    def __init__(self,reservoir_limit):
        self.callbacks=0;self.accepted=0;self.rejections=Counter();self.callers=Counter()
        self.first_receive=None;self.last_receive=None;self.first_wall=None;self.last_wall=None
        self.first_source=None;self.last_source=None;self.duplicates=0;self.regressions=0;self.negative_ages=0
        self.gaps=ScalarSummary(reservoir_limit);self.source_age=ScalarSummary(reservoir_limit)

    def accept(self,stamp_ns,receive_ns,wall_ns):
        self.accepted+=1
        if self.first_receive is None:
            self.first_receive=receive_ns;self.first_wall=wall_ns;self.first_source=stamp_ns
        else:
            self.gaps.add((receive_ns-self.last_receive)/1e6)
            self.duplicates+=stamp_ns==self.last_source;self.regressions+=stamp_ns<self.last_source
        age=(wall_ns-stamp_ns)/1e6
        self.source_age.add(age);self.negative_ages+=age<0
        self.last_receive=receive_ns;self.last_wall=wall_ns;self.last_source=stamp_ns

    def report(self,start_ns,end_ns):
        duration=max(0.,(end_ns-start_ns)/1e9)
        active=(self.last_receive-self.first_receive)/1e9 if self.accepted>1 else 0.
        begin=(self.first_receive-start_ns)/1e6 if self.accepted else duration*1000
        tail=(end_ns-self.last_receive)/1e6 if self.accepted else duration*1000
        return dict(callbacks=self.callbacks,accepted_callbacks=self.accepted,rejected=dict(self.rejections),
            source_callers=dict(self.callers),arrival_hz=(self.accepted-1)/active if active>0 else None,
            accepted_callbacks_per_window_second=self.accepted/duration if duration>0 else None,
            interarrival_gap_ms=self.gaps.report(),initial_silence_ms=max(0.,begin),
            final_silence_ms=max(0.,tail),maximum_gap_including_window_boundaries_ms=max(0.,begin,tail,self.gaps.maximum or 0.),
            source_age_including_clock_offset_ms=self.source_age.report(),negative_source_age_samples=self.negative_ages,
            consecutive_duplicate_source_stamps=self.duplicates,source_stamp_regressions=self.regressions,
            first_receive_wall_ns=self.first_wall,last_receive_wall_ns=self.last_wall,
            first_source_stamp_ns=self.first_source,last_source_stamp_ns=self.last_source)


class TelemetryMetrics:
    def __init__(self,run_id,start_ns,start_wall_ns,streams=('actual','vio'),
                 expected_caller='/distributed_telemetry_observer',pair_rate=50.,pair_max_age=.2,reservoir_limit=8192):
        if not streams or any(key not in ('actual','vio') for key in streams):raise ValueError('Unknown telemetry stream')
        if not all(math.isfinite(v) and v>0 for v in (pair_rate,pair_max_age)):raise ValueError('Invalid sampler settings')
        self.run_id=run_id;self.start_ns=start_ns;self.start_wall_ns=start_wall_ns
        self.expected_caller=expected_caller;self.pair_rate=pair_rate;self.pair_max_age_ns=round(pair_max_age*1e9)
        self.lock=threading.Lock();self.closed=False
        self.streams={(robot,key):StreamStats(reservoir_limit) for robot in (0,1) for key in dict.fromkeys(streams)}
        self.latest_actual={};self.pair_counts=Counter();self.pair_difference=ScalarSummary(reservoir_limit)
        self.pair_tick_gaps=ScalarSummary(reservoir_limit);self.last_pair_tick=None

    def ingest(self,robot,key,stamp_ns,receive_ns,wall_ns,caller):
        with self.lock:
            if self.closed:return False
            stats=self.streams[robot,key];stats.callbacks+=1;stats.callers[str(caller)]+=1
            if caller!=self.expected_caller:stats.rejections['unexpected_caller']+=1;return False
            if type(stamp_ns) is not int or stamp_ns<=0:stats.rejections['invalid_source_stamp']+=1;return False
            if type(receive_ns) is not int or receive_ns<self.start_ns or (stats.last_receive is not None and receive_ns<stats.last_receive):
                stats.rejections['receipt_clock_regression']+=1;return False
            stats.accept(stamp_ns,receive_ns,wall_ns)
            if key=='actual':self.latest_actual[robot]=(stamp_ns,receive_ns)
            return True

    def sample_pair(self,now_ns):
        with self.lock:
            if self.closed:return
            self.pair_counts['ticks']+=1
            if self.last_pair_tick is not None:self.pair_tick_gaps.add((now_ns-self.last_pair_tick)/1e6)
            self.last_pair_tick=now_ns
            if 'actual' not in {key for _,key in self.streams}:self.pair_counts['actual_not_subscribed']+=1;return
            if len(self.latest_actual)!=2:self.pair_counts['missing_actual_ticks']+=1;return
            if any(now_ns-received<0 or now_ns-received>self.pair_max_age_ns for _,received in self.latest_actual.values()):
                self.pair_counts['stale_receipt_ticks']+=1;return
            difference=abs(self.latest_actual[0][0]-self.latest_actual[1][0])
            self.pair_counts['eligible_ticks']+=1;self.pair_counts['below_50ms']+=difference<50000000
            self.pair_counts['below_100ms']+=difference<100000000;self.pair_difference.add(difference/1e6)

    def finish(self,end_ns,end_wall_ns,reason='duration_elapsed'):
        with self.lock:
            self.closed=True;duration=max(0.,(end_ns-self.start_ns)/1e9)
            eligible=self.pair_counts['eligible_ticks'];ticks=self.pair_counts['ticks']
            pair=dict(method='fixed-rate sampler ticks using latest received source stamp from each robot',
                sample_basis='sampler_ticks; not callback pairs, synchronized acquisitions or interpolation',
                target_sampler_hz=self.pair_rate,actual_sampler_hz=ticks/duration if duration>0 else None,
                maximum_receipt_age_s=self.pair_max_age_ns/1e9,counters=dict(self.pair_counts),
                eligible_tick_fraction=eligible/ticks if ticks else None,
                absolute_source_time_difference_ms=self.pair_difference.report(),
                fraction_below_50ms=self.pair_counts['below_50ms']/eligible if eligible else None,
                fraction_below_100ms=self.pair_counts['below_100ms']/eligible if eligible else None,
                threshold_denominator='eligible sampler ticks only; missing/stale coverage reported separately',
                sampler_tick_gap_ms=self.pair_tick_gaps.report(),includes_inter_robot_clock_offset=True)
            return dict(run_id=self.run_id,observer_only=True,feeds_slam=False,raw_messages_saved=False,
                poses_saved=False,expected_source_caller=self.expected_caller,
                started_wall_ns=self.start_wall_ns,finished_wall_ns=end_wall_ns,observed_duration_s=duration,
                stop_reason=reason,definitions=dict(
                    arrival_hz='(accepted callbacks - 1) / monotonic time between first and last accepted callback',
                    gaps='interarrival gaps exclude boundaries; initial/final silence and full-window maximum are separate',
                    source_age='hub callback wall time minus source Header.stamp; includes host clock offset, processing, queues and transport; NOT one-way network latency',
                    negative_age='retained and counted; never silently clamped to zero',
                    percentiles='exact up to reservoir limit, otherwise bounded uniform reservoir; count/min/max/mean remain cumulative'),
                streams={'sim_%d/%s'%(robot,key):stats.report(self.start_ns,end_ns) for (robot,key),stats in self.streams.items()},
                actual_pair=pair)


def argument_parser():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True);parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--output',type=Path,help='Optional hub JSON file; report is also printed to stdout')
    parser.add_argument('--streams',nargs='+',choices=('actual','vio'),default=['actual','vio'])
    parser.add_argument('--pair-rate',type=float,default=50.);parser.add_argument('--pair-max-age',type=float,default=.2)
    parser.add_argument('--expected-caller',default='/distributed_telemetry_observer',
        help='For the legacy combined observer only, explicitly choose /distributed_result_observer')
    parser.add_argument('--reservoir-limit',type=int,default=8192)
    return parser


def main(argv=None):
    parser=argument_parser();args=parser.parse_args(argv)
    if not all(math.isfinite(v) and v>0 for v in (args.duration,args.pair_rate,args.pair_max_age)) or args.reservoir_limit<1:
        parser.error('Duration, sampler rates/ages and reservoir limit must be positive and finite')
    import rospy
    from nav_msgs.msg import Odometry
    rospy.init_node('distributed_telemetry_metrics',anonymous=True,disable_signals=True)
    stop=threading.Event();reason=['duration_elapsed']
    def interrupted(signum,*_):reason[0]='signal_'+str(signum);stop.set()
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,interrupted)
    start=time.monotonic_ns();metrics=TelemetryMetrics(args.run_id,start,time.time_ns(),args.streams,
        args.expected_caller,args.pair_rate,args.pair_max_age,args.reservoir_limit)
    subscriptions=[]
    for robot,key in metrics.streams:
        def receive(message,owner=robot,stream=key):
            receive_ns=time.monotonic_ns();wall_ns=time.time_ns()
            caller=(getattr(message,'_connection_header',None) or {}).get('callerid')
            metrics.ingest(owner,stream,int(message.header.stamp.to_nsec()),receive_ns,wall_ns,caller)
        subscriptions.append(rospy.Subscriber('/observer/sim_%d/%s'%(robot,key),Odometry,receive,
            queue_size=200,tcp_nodelay=True))
    deadline=start+round(args.duration*1e9);period=max(1,round(1e9/args.pair_rate));next_tick=time.monotonic_ns()
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            now=time.monotonic_ns()
            if now>=deadline:break
            metrics.sample_pair(now);next_tick+=period
            if next_tick<now-period:next_tick=now+period  # Skip missed ticks rather than fabricating catch-up samples.
            stop.wait(max(0.,min(next_tick,deadline)-time.monotonic_ns())/1e9)
        if rospy.is_shutdown() and reason[0]=='duration_elapsed':reason[0]='ros_shutdown'
    finally:
        report=metrics.finish(time.monotonic_ns(),time.time_ns(),reason[0]);report['ros_master_uri']=os.environ.get('ROS_MASTER_URI')
        for subscription in subscriptions:subscription.unregister()
        data=json.dumps(report,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n'
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            temporary=args.output.with_suffix(args.output.suffix+'.tmp');temporary.write_text(data);temporary.replace(args.output)
        print(data,end='',flush=True)


if __name__=='__main__':main()
