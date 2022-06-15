# DoIt

[DoIt](https://pydoit.org/) is a simple python library for handling tasks and
providing a nice CLI interface.  Another alternative is something like
[Scons](https://scons.org/) (more for doing build systems) or
[Celery](https://docs.celeryq.dev/en/stable/getting-started/first-steps-with-celery.html#first-steps)
(more for distributed systems).

I'm currently looking at using DoIt to replace some python code I wrote for simple task dependency management.  I'd also like to extend my task dependencies to using multiple machines over SSH or some other communication protocol (like MPI).

## Tutorial

The `tutorial-1` directory follows the corresponding [tutorial from DoIt](https://pydoit.org/tutorial-1.html).
