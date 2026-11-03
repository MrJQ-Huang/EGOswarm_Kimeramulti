.PHONY: test
test:
	python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'
	python3 -m unittest discover -s experiment/three_host_20261102/tests -p 'test_*.py'
