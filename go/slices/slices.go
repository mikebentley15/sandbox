package main

import "fmt"

type InterfaceList []interface{}
type IntList []int

//func (x []interface{}) last(x []interface{}) int {
//	return len(x) - 1
//}
func last_1(x []interface{}) interface{} {
	return len(x) - 1
}
func last_2(x []int) int {
	return x[len(x)-1]
}
func (x IntList) last_3() int {
	return x[len(x)-1]
}
func (x InterfaceList) last_4() interface{} {
	return x[len(x)-1]
}

// NOTE: the commented out lines are not runnable, i.e. syntax errors

func main() {
	leader := 4
	a := make([]int, 10)
	var b []interface{}
	for i := range a {
		a[i] = i
	}
	printSlice("a", a)
	fmt.Printf("leader %v\n", leader)
	fmt.Printf("len(b) %v\n", len(b))
	//fmt.Printf("a[-1]: %v\n", last_1(a))
	fmt.Printf("a[-1]: %v\n", last_2(a))
	//fmt.Printf("a[-1]: %v\n", a.last_3())
	//fmt.Printf("a[-1]: %v\n", a.last_4())
	var c IntList
	c = a
	fmt.Printf("c[-1]: %v\n", c.last_3())
	//fmt.Printf("c[-1]: %v\n", c.last_4())
	for _, val := range c {
		b = append(b, val)
	}
	fmt.Printf("b[-1]: %v\n", last_1(b))
	//fmt.Printf("b[-1]: %v\n", b.last_4())
	//fmt.Printf("b[-1]: %v\n", b.(InterfaceList).last_4())
	//d := b.(InterfaceList)
	var d InterfaceList
	d = b
	fmt.Printf("d[-1]: %v\n", d.last_4())
	// Neither of these work:
	//fmt.Println("range a:", 1:10)
	//fmt.Printf("a[-1]", a[-1])
	//a[:] = 1:10
}

func printSlice(s string, x []int) {
	fmt.Printf("%s len=%d cap=%d %v\n", s, len(x), cap(x), x)
}
