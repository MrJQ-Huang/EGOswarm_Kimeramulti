#!/usr/bin/env python3
"""Fetch pinned upstream source into new directories and apply recorded patches.

This does not build, install, start experiments, or modify an existing checkout.
Use --list to inspect the plan without writing files or accessing the network.
"""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess


def within(root, relative):
    path = (root / relative).resolve()
    if path == root or root not in path.parents:
        raise ValueError('Path escapes its source root: ' + relative)
    return path


def git(directory, *arguments):
    subprocess.run(['git', '-C', str(directory), *arguments], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', choices=('amov', 'uros'), required=True)
    parser.add_argument('--destination', type=Path)
    parser.add_argument('--component', action='append', help='Repository id to fetch; repeatable.')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    dependencies = Path(__file__).resolve().parents[1] / 'dependencies'
    manifest = json.loads((dependencies / 'source_manifest.json').read_text(encoding='utf-8'))
    chosen = set(args.component or [])
    repositories = [r for r in manifest['repositories'] if not chosen or r['id'] in chosen]
    missing = chosen - {r['id'] for r in repositories}
    if missing:
        parser.error('Unknown repository ids: ' + ', '.join(sorted(missing)))
    for repo in repositories:
        if not re.fullmatch(r'https://github\.com/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+(?:\.git)?', repo['url']):
            raise ValueError('Expected a credential-free GitHub HTTPS source URL')
        if not re.fullmatch(r'[0-9a-f]{40}', repo['commit']):
            raise ValueError('Expected an immutable commit SHA for ' + repo['id'])
        for patch in repo['patches'][args.host]:
            if not within(dependencies, patch).is_file():
                raise ValueError('Missing recorded patch: ' + patch)
    if args.list:
        print(json.dumps({'host': args.host, 'repositories': repositories,
                          'post_fetch_notes': manifest['post_fetch_notes']}, indent=2))
        return
    if args.destination is None:
        parser.error('--destination is required unless --list is used')
    root = args.destination.resolve()
    targets = [(repo, within(root, repo['directory'])) for repo in repositories]
    if len({path for _, path in targets}) != len(targets):
        raise ValueError('Duplicate repository destination')
    for _, path in targets:
        if path.exists():
            raise FileExistsError('Refuse to overwrite existing source: ' + str(path))
    extra = []
    if not chosen:
        for entry in manifest['extra_source_directories']:
            if args.host in entry['hosts']:
                source = within(dependencies, entry['source'])
                destination = within(root, entry['destination'])
                for child in source.iterdir():
                    target = within(root, str((destination / child.name).relative_to(root)))
                    if target.exists() or any(target == path for _, path in targets):
                        raise FileExistsError('Refuse wrapper destination collision: ' + str(target))
                    extra.append((child, target))
    root.mkdir(parents=True, exist_ok=True)
    for repo, path in targets:
        path.mkdir(parents=True)
        subprocess.run(['git', 'init', str(path)], check=True)
        git(path, 'remote', 'add', 'origin', repo['url'])
        git(path, 'fetch', '--depth', '1', 'origin', repo['commit'])
        git(path, 'checkout', '--detach', 'FETCH_HEAD')
        for relative in repo['patches'][args.host]:
            patch = within(dependencies, relative)
            git(path, 'apply', '--check', str(patch))
            git(path, 'apply', str(patch))
        print('Prepared ' + repo['id'], flush=True)
    for source, destination in extra:
        if source.is_dir():
            shutil.copytree(source, destination)
        else:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
    print('\n'.join(manifest['post_fetch_notes']))


if __name__ == '__main__':
    main()
