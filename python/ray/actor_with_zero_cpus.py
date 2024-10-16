import os

import ray

ray.init()

@ray.remote(num_cpus=0)
class ActorZero:
    def __init__(self):
        print("ActorZero.__init__()")

    def run(self):
        print("ActorZero.run()")

@ray.remote
class ActorOne:
    def __init__(self):
        print("ActorOne.__init__()")

    def run(self):
        print("ActorOne.run()")

@ray.remote(num_cpus=2)
class ActorTwo:
    def __init__(self):
        print("ActorTwo.__init__()")

    def run(self):
        print("ActorTwo.run()")

@ray.remote
def run_actor(name, actor):
    print(f"run_actor({name})")
    actor.run.remote()

def test_actor(name, actor):
    print(f"test_actor({name})")
    ray.get(actor.run.remote())
    ray.get([run_actor.remote(name, actor) for _ in range(10)])

print(f"Main PID: {os.getpid()}")

actors = {
    "ActorZero_1": ActorZero.remote(),
    "ActorZero_2": ActorZero.remote(),
    "ActorOne_1": ActorOne.remote(),
    "ActorOne_2": ActorOne.remote(),
    "ActorTwo_1": ActorTwo.remote(),
    "ActorTwo_2": ActorTwo.remote(),
}

for name, actor in actors.items():
    test_actor(name, actor)
