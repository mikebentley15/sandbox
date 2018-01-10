package main

import (
	"fmt"
	"sort"
)

type stack []int

func (s *stack) Print()      { fmt.Printf("Stack(%v)\n", *s) }
func (s *stack) Sort()       { sort.Ints(*s) }
func (s *stack) Empty() bool { return len(*s) == 0 }
func (s *stack) Peek() int   { return (*s)[len(*s)-1] }
func (s *stack) Put(x int)   { (*s) = append((*s), x) }
func (s *stack) Pop() int {
	d := (*s)[len(*s)-1]
	(*s) = (*s)[:len(*s)-1]
	return d
}

func main() {
	var s stack
	s.Print()

	s.Put(1)
	s.Print()

	fmt.Printf("peek:  %v\n", s.Peek())
	s.Print()

	s.Put(5)
	fmt.Printf("pop:   %v\n", s.Pop())
	s.Print()

	s.Put(3)
	s.Put(2)
	s.Put(4)
	s.Put(5)
	s.Print()

	s.Sort()
	sort.Ints(s)
	s.Print()
}
