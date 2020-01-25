from pygments.formatters.terminal256 import EscapeSequence, Terminal256Formatter
import sys
_formatter = Terminal256Formatter()

class Token:
    def __init__(self, message='', color=None, bgcolor=None, bold=False,
                 underline=False):
        self.message = message
        self.color = color
        self.bgcolor = bgcolor
        self.bold = bold
        self.underline = underline

def print_tokens(tokens, file=sys.stdout):
    for token in tokens:
        colorprint(token.message, token.color, file=file,
                   bgcolor=token.bgcolor, bold=token.bold,
                   underline=token.underline)

def colorprint(message, color, file=sys.stdout, bgcolor=None, bold=False,
               underline=False,):
    '''
    Print a message with ANSI codes for the given colors

    @param message (str): text to be color printed
    @param color (str): hex representation of the color
        (e.g., blue is '0000ff')
    @param bgcolor (str): hex representation of the background color
        (default is None)
    @param file (stream): where to output the color message
        (default is sys.stdout)
    @param bold (bool): True means output the message in bold font
        (default is False)
    @param underline (bool): True means underline the message
        (default is False)

    @return None
    '''
    escape = EscapeSequence()
    if color: escape.fg = _formatter._color_index(color)
    if bgcolor: escape.bg = _formatter._color_index(bgcolor)
    escape.bold = bold
    escape.underline = underline
    file.write(escape.color_string())
    file.write(message)
    file.write(escape.reset_string())

def main():
    T = Token
    print_tokens([
        T('hello', underline=True),
        T(' '),
        T('my friend', color='0000ff', bold=True),
        T('!!\n', bold=True),
        ])

if __name__ == '__main__':
    main()
