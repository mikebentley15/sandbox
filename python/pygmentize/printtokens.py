from pygments import formatters, token
import sys

formatter = formatters.find_formatter_class('terminal')(bg='dark')
formatter.format((
        (token.Name.Tag, '  my name:'),
        (token.Whitespace, '  '),
        (token.Text, 'Michael'),
        (token.Whitespace, '\n'),
        (token.Name.Tag, 'your name:'),
        (token.Whitespace, '  '),
        (token.Text, 'Sammy'),
        (token.Whitespace, '\n'),
    ), sys.stdout)
