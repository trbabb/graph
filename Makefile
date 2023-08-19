CC        = clang++
AR        = ar
PREFIX    = /usr/local
INCLUDE_DIR = graph

H_FILES  = $(wildcard $(INCLUDE_DIR)/*.h)
INSTALL_INC_DIR = $(PREFIX)/include/$(INCLUDE_DIR)


all: install

docs:
	mkdir -p doc/gen
	doxygen

clean:
	rm -rf ./build/*
	rm -rf ./doc/gen/html

install: $(H_FILES)
	install -d $(INSTALL_INC_DIR)
	for file in $(H_FILES); do \
		install -m 0644 $$file $(INSTALL_INC_DIR); \
	done
