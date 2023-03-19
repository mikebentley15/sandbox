import cmd

class CommandWrapPrompt(cmd.Cmd):
    intro = 'Wrap a function call with a command-prompt'
    prompt = 'command-wrap> '

    def __init__(self, command):
        self.command = command
        super().__init__()

    def default(self, line):
        if line == 'EOF':
            print('\n\n  Goodbye!\n')
            return True # stop the loop
        self.command(line)
        return False # do not stop
