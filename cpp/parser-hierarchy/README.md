# A C++ Lexer and Parser for C++ Hierarchy

The purpose of this project is to generate a hierarchy for C++ source code for
use with hierarchical delta debugging.  Therefore, we only car about part of
the semantics of the C++ language.  Specifically the nesting of scope.


## Output

The input is a C++ source file.  No macros statements will be expanded,
including include statements.  If you want include statements to be included,
simply use a compiler to do the preprocessing portion (which can be done with
the `cpp` command).

The hierarchical structure will be done with indentation.  All whitespace will
be collapsed, all comments removed, and whitespace will be added in to specify
the hierarchical structure.  Each line is an indivisible piece (that could be
used with delta debugging directly).  The hierarchy will be set by indentation.
The line before and after an indented block will be part of that hierarchical
piece, but the line before and after only specify the block, but are not part
of the block.

Here is an example input file:

```c++
#include <cstdio>

#ifndef __MAC__
#define max(x, y) \
  (x > y ? \
   x : y)
#endif // end of #ifndef __MAC__

// main function
int
main(int argc,
     char** argv)
{
  bool extranewline = true;
  // print three hello worlds in one line
  for (int i = 0; i < 3; i++)
    {
      printf("Hello world! ; ");
    }

  // end in a newline
  if (extranewline)
    printf("\n");

  // success!
  return 0;
}
```

And here would be the output after parsing:

```c++
#include <cstdio>
#ifndef __MAC__
#define max(x, y) (x > y ? x : y)
#endif
int main ( int argc ,char **argv ) {
 bool extranewline =true;
 for ( int i = 0 ; i < 3 ; i++ ) {
  printf ( "Hello world! ; " ) ;
 }
 if ( extranewline ) printf ( "\n" ) ;
 return 0 ;
}
```

There are two hierarchies here, first the `main` function that could be removed
as a whole, or inside, each of the statements.  Within the `main` function is
another level which is the `for` loop.  This output should be easy to parse
into a hierarchy simply by checking indentation level.  The indentation is one
space per indentation level, so it's easy.

## Grammar

Below are the grammar rules that are implemented in this lexer and parser.  The
parser is implemented as a recursive descent parser.

```
file            := {element | eaten}.
eaten           := (whitespace | comment).
whitespace      := {" " | "\t" | "\n"}.
comment         := line_comment | multi_comment.
line_comment    := "//" {anything} "\n".
multi_comment   := "/*" (^"*/") "*/".
element         := (macro | statement | block).
macro           := "#" {piece "\\\n"} "\n".
statement       := statement_inner ";".
statement_inner := {pstatement | piece}.
pstatement      := "(" {statement_inner | ";"} ")".
piece           := (literal | identifier | operator).
literal         := (number | string | character).
block           := [statement_inner] "{" {element} "}".
number          := (0-9) {(a-zA-Z0-9.)}.
string          := '"' {(^'\"')} '"'.
character       := "'" {(^"\'")} "'".
identifier      := (a-zA-Z_) {(a-zA-Z0-9_)}.
operator        := (+=/?<>~!@#$^&*,|[].:-).
```

### Scanner Grammar

```
file            := {eaten | token}.
eaten           := (whitespace | comment).
whitespace      := {" " | "\t" | "\n"}.
comment         := line_comment | multi_comment.
line_comment    := "//" (^"\n") "\n".
multi_comment   := "/*" (^"*/") "*/".
token           := (operator | identifier | literal | macro |
                    "(" | ")" | "{" | "}" | ";" ).
operator        := (+=/?<>~!@$^&*,|[].:-).
identifier      := (a-zA-Z_) {(a-zA-Z0-9_)}.
literal         := (number | string | character).
number          := (0-9) {(a-zA-Z0-9.)}.
string          := '"' {(^'\"')} '"'.
character       := "'" {(^"\'")} "'".
macro           := "#" {(^"\\\n")} "\n".
```

Only the tokens are returned.  The `eaten` category is not given to the parser.

### Parser Grammar

TODO: how do we handle template functions, classes, and structs?

```
file            := {element}.
element         := (label | macro | statementblock).
label           := ("public" | "protected" | "private") ":"
statementblock  := "template"
                     {(pstatement | piece) that is not "class" or "struct"}
                     (class | ";" | block) |
                   class | enum | union | block | typedef |
                   statement_inner [";" | block]
class           := ("class" | "struct" | "union") [statement_inner]
                   (semiblock | ";").
enum            := "enum" [statement_inner] (enumblock | ";").
typedef         := "typedef" (enum | union | class |
                              statement_inner (";" | block))
statement_inner := (pstatement | piece) {pstatement | piece}.
pstatement      := "(" {statement_inner | ";"} ")".
piece           := (literal | identifier | operator).
semiblock       := basic_block
                   [{operator} identifier {"," {operator} identifier}] ";"
block           := basic_block [";"]
basic_block     := "{" {element} "}".
enumblock       := braceinit [identifier {"," identifier}] ";".
braceinit       := "{" [statement_inner] "}"
```

