import click

@click.group()
@click.option("--required", required=True, type=click.STRING)
def cli(required):
    # do something with required
    pass

@cli.command()
@click.argument("filepath", type=click.File("r"), required=True)
def sub(filepath):
    # do something with filepath
    pass

if __name__ == "__main__":
    cli()
