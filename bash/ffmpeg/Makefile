SAMPLE    := sample.webm
CROPPED_1 := $(SAMPLE:.webm=-crop1.mp4)
CROPPED_2 := $(SAMPLE:.webm=-crop2.mp4)
VSPLIT    := $(SAMPLE:.webm=-vsplit.mp4)

GENERATED := $(CROPPED_1)
GENERATED += $(CROPPED_2)
GENERATED += $(VSPLIT)

.PHONY: default all clean
default: all
all: $(GENERATED)
clean:
	$(RM) $(GENERATED)

$(CROPPED_1): $(SAMPLE) crop.bash
	./crop.bash $(SAMPLE) 1000 1080 200 0 $(CROPPED_1)

$(CROPPED_2): $(SAMPLE) crop.bash
	./crop.bash $(SAMPLE) 920 1080 800 0 $(CROPPED_2)

$(VSPLIT): $(CROPPED_1) $(CROPPED_2) vsplit.bash
	./vsplit.bash $(CROPPED_1) $(CROPPED_2) $(VSPLIT)
