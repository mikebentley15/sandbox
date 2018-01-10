from distutils.core import setup, Extension

spam_module = Extension(
        'spam',
        sources=[
            'spammodule.c',
            'spamfunctions.c',
            'Spam.c',
            'Noddy.c',
            ],
        )

setup(name='spam',
      version='1.0',
      description='Spam module example from python documentation',
      ext_modules=[spam_module])
