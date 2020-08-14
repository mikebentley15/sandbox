LOG_FILE       := my-logfile.log
LOG_EVENT       = echo '{"date": "$(shell date)", "time": "$(shell date +%s%N)", "type": "$1", "message": "$2"}' >> $(LOG_FILE)

.PHONY: all clean
all:
	$(call LOG_EVENT,MyEvent,message information)

clean:
	rm -f $(LOG_FILE)
