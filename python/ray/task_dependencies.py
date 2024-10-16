import ray

ray.init()

@ray.remote
def square(x: int) -> int:
    """If a ray object ref is passed into remote(), then its associated object will be sent instead."""
    return x*x

@ray.remote
def reduce(vals: list[ray.ObjectRef]) -> int:
    """It doesn't work for a list of ray object refs.  You'll need to call get yourself."""
    return sum(ray.get(vals))

print("square(square(square(2))):", ray.get(square.remote(square.remote(square.remote(2)))))
print("reduce([square(x) for x in range(10)]): ", ray.get(reduce.remote([square.remote(x) for x in range(10)])))
