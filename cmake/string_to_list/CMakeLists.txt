cmake_minimum_required(VERSION 3.5)
project(string_to_list)

set(string_var "this is a string variable")
list(LENGTH string_var len)
message("string_var = ${string_var}")
message("len(string_var) = ${len}")

list(APPEND string_var "another val")
list(LENGTH string_var len)
message("string_var = ${string_var}")
message("len(string_var) = ${len}")

string(REPLACE " " ";" list_var "${string_var}")
list(LENGTH list_var len)
message("list_var = ${list_var}")
message("len(list_var) = ${len}")

set(string_var "this string;has a semicolon")
list(LENGTH string_var len)
message("string_var = ${string_var}")
message("len(string_var) = ${len}")

set(list_var
  "this"
  "string\\;has"
  "a"
  "semicolon"
)
list(LENGTH list_var len)
message("list_var = ${list_var}")
message("len(list_var) = ${len}")


