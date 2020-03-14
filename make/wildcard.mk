DIRS := /tmp/a
DIRS += /tmp/a/b
DIRS += /tmp/a/c
DIRS += /tmp/a/d
DIRS += /tmp/a/b/e
DIRS += /tmp/a/b/f
DIRS += /tmp/a/b/g
DIRS += /tmp/a/c/h
DIRS += /tmp/a/c/i
DIRS += /tmp/a/c/j
.PHONY: dirs
dirs: $(DIRS)

$(DIRS): $(dir $@)
	mkdir $@

$(info "All dirs: " $(wildcard /tmp/a/*/*))

.PHONY: clean
clean:
	rm -rf $(DIRS)
