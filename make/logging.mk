LOG_FILE_BASE  := my-logfile
LOGFILE        ?= $(LOG_FILE_BASE)-$$PPID.log
LOG_EVENT       = echo '{"date": "'$$(date)'", "time": "'$$(date +%s%N)'", "type": "$1", "message": "$2"}'

TMPDIR         := tmpdir
FILES          :=
FILES          += $(TMPDIR)/01.txt
FILES          += $(TMPDIR)/02.txt
FILES          += $(TMPDIR)/03.txt
FILES          += $(TMPDIR)/04.txt
FILES          += $(TMPDIR)/05.txt
FILES          += $(TMPDIR)/06.txt
FILES          += $(TMPDIR)/07.txt
FILES          += $(TMPDIR)/08.txt
FILES          += $(TMPDIR)/09.txt
FILES          += $(TMPDIR)/11.txt
FILES          += $(TMPDIR)/12.txt
FILES          += $(TMPDIR)/13.txt
FILES          += $(TMPDIR)/14.txt
FILES          += $(TMPDIR)/15.txt
FILES          += $(TMPDIR)/16.txt
FILES          += $(TMPDIR)/17.txt
FILES          += $(TMPDIR)/18.txt
FILES          += $(TMPDIR)/19.txt
FILES          += $(TMPDIR)/21.txt
FILES          += $(TMPDIR)/22.txt
FILES          += $(TMPDIR)/23.txt
FILES          += $(TMPDIR)/24.txt
FILES          += $(TMPDIR)/25.txt
FILES          += $(TMPDIR)/26.txt
FILES          += $(TMPDIR)/27.txt
FILES          += $(TMPDIR)/28.txt
FILES          += $(TMPDIR)/29.txt
FILES          += $(TMPDIR)/31.txt
FILES          += $(TMPDIR)/32.txt
FILES          += $(TMPDIR)/33.txt
FILES          += $(TMPDIR)/34.txt
FILES          += $(TMPDIR)/35.txt
FILES          += $(TMPDIR)/36.txt
FILES          += $(TMPDIR)/37.txt
FILES          += $(TMPDIR)/38.txt
FILES          += $(TMPDIR)/39.txt
FILES          += $(TMPDIR)/41.txt
FILES          += $(TMPDIR)/42.txt
FILES          += $(TMPDIR)/43.txt
FILES          += $(TMPDIR)/44.txt
FILES          += $(TMPDIR)/45.txt
FILES          += $(TMPDIR)/46.txt
FILES          += $(TMPDIR)/47.txt
FILES          += $(TMPDIR)/48.txt
FILES          += $(TMPDIR)/49.txt
FILES          += $(TMPDIR)/50.txt

ifneq ($(LOGFILE),)

.PHONY: default
default: all

.PHONY: %

%:
	@$(MAKE) $@ --output-sync=line LOGFILE="" -f $(firstword $(MAKEFILE_LIST)) \
		| tee -a $(LOGFILE) \
		| grep -v "^\\{"
	@grep "^\\{" $(LOGFILE) > tmp.log
	@mv tmp.log $(LOGFILE)

else

.PHONY: all clean
all: $(FILES)
	@$(call LOG_EVENT,MyEvent,message information)

$(TMPDIR):
	mkdir -p $@

$(FILES): | $(TMPDIR)
	@$(call LOG_EVENT,MyEvent,01: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,02: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,03: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,04: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,05: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,06: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,07: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,08: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,09: creating $@)
	@sleep 0.02
	@$(call LOG_EVENT,MyEvent,10: creating $@)
	@sleep 0.02
	touch $@

clean:
	rm -f $(LOG_FILE_BASE)-*.log
	rm -rf $(TMPDIR)

endif
