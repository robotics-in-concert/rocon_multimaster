#include $(shell rospack find mk)/cmake.mk

all:
	@echo "Nothing to see here - valid make targets are:"
	@echo ""
	@echo "  sources : export the github sources to the correct ros directory."
	@echo "  clone   : clone the yujinrobot patched repo to work on it."

sources:
	rm -rf src/*
	@echo "Exporting sources from the git repo into our ros directory."
	git clone https://github.com/yujinrobot/redis-py.git
	cd redis-py; git archive master | tar -x -C ../src
	rm -rf redis-py

clone:
	git clone https://github.com/yujinrobot/redis-py.git
