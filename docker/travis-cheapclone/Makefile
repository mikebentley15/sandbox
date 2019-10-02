DOCKER_ARGS    :=
DOCKER_ARGS    += --rm
DOCKER_ARGS    += -it
DOCKER_ARGS    += -e HOME=$(HOME)
DOCKER_ARGS    += -v "$(HOME):$(HOME)/"
DOCKER_ARGS    += -v /etc/group:/etc/group:ro
DOCKER_ARGS    += -v /etc/passwd:/etc/passwd:ro
DOCKER_ARGS    += --security-opt seccomp=unconfined
DOCKER_ARGS    += -e DISPLAY=$(DISPLAY)
DOCKER_ARGS    += -v /tmp/.X11-unix:/tmp/.X11-unix
DOCKER_ARGS    += -u $(shell -id -u $(USER)):$(shell id -g $(USER))

.PHONY: default build-travis build-flit run-travis run-flit

default: build-flit

build-travis:
	docker build \
	  --build-arg myuser=$(shell whoami) \
	  -t travis-cheapclone \
	  -f Dockerfile \
	  ./

build-flit: build-travis
	docker build -t travis-cheapclone:flit -f Dockerfile.flit .

run-travis: build-travis
	xhost +"local:docker@"
	docker run $(DOCKER_ARGS) travis-cheapclone:latest

run-flit: build-flit
	xhost +"local:docker@"
	docker run $(DOCKER_ARGS) travis-cheapclone:flit
