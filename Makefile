.PHONY: test
test:
	python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'
