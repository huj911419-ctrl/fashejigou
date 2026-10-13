# Start here: plain `make` prints the available workflows.
# Firmware compilation is kept in build_scripts/firmware.mk.
ifneq ($(origin ROBOT),undefined)
include build_scripts/firmware.mk
else
.DEFAULT_GOAL := help
.PHONY: help gimbal friction sentry infantry test check clean
PYTHON ?= python3
JOBS ?= 4

help:
	@printf '%s\n' '选择要做的事（在工程根目录运行）：' \
	  '  make gimbal      编译独立云台' \
	  '  make friction    编译独立摩擦轮' \
	  '  make test        运行电脑上的离线测试，无需接板' \
	  '  make check       检查代码层级和文档路径' \
	  '  make sentry      编译旧哨兵应用' \
	  '  make infantry    编译旧步兵应用' \
	  '  make clean       清理默认旧哨兵构建；其他应用用 make ROBOT=<名称> clean' \
	  '详细入口：README.md → 从这里开始.md → docs/README.md' \
	  '旧命令 make -f Makefile.gimbal/friction、make ROBOT=sentry/infantry 继续有效。'

gimbal:
	$(MAKE) -f Makefile.gimbal
friction:
	$(MAKE) -f Makefile.friction
sentry infantry:
	$(MAKE) ROBOT=$@
test:
	$(PYTHON) tests/verify_offline.py --jobs $(JOBS)
check:
	$(PYTHON) tests/check_architecture.py
	$(PYTHON) tests/check_layout.py
clean:
	$(MAKE) ROBOT=sentry clean
endif
