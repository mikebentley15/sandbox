package main

import "fmt"

type S struct {
	id  int
	val string
}

func (s S) String() string {
	return fmt.Sprintf("S{%v, %v}", s.id, s.val)
}

func main() {
	fmt.Println()

	a := S{1, "hello"}
	b := a
	b.val = "b"
	a.val = "a"
	fmt.Println("a: ", a)
	fmt.Println("b: ", b)
	fmt.Println()

	c := &a

	defer fmt.Println()
	defer func() { fmt.Println("defered func() {c}: ", c) }()
	defer func() { fmt.Println("defered func() {b}: ", b) }()
	defer func() { fmt.Println("defered func() {a}: ", a) }()
	defer fmt.Println()
	defer fmt.Println("defered c: ", c)
	defer fmt.Println("defered &b: ", &b)
	defer fmt.Println("defered &a: ", &a)
	defer fmt.Println()
	defer fmt.Println("defered c: ", c)
	defer fmt.Println("defered b: ", b)
	defer fmt.Println("defered a: ", a)

	c.val = "c"
	fmt.Println("a: ", a)
	fmt.Println("b: ", b)
	fmt.Println("c: ", c)
	fmt.Println()
}
