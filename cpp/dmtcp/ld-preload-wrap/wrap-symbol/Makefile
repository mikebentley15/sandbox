# If other .o files call foo, they will also be interposed to foo_wrapper.o.
# This version correctly handles a def. of foo and call to foo in the same file.
# But if foo is resursive, this version will only wrap the first call to foo.

# If we wanted to extend this to allow chaining (e.g., for foo to call libc:foo,
# Then we would redefine foo to __wrapper_foo in orig2.o.

.PHONY: run clean
run: a.out
	@ echo ""
	@ echo =====================================
	@ echo ==========BEGIN EXECUTION============
	@ echo =====================================
	./a.out

a.out: orig.o foo_wrapper.o
	gcc orig.o foo_wrapper.o -o a.out

orig.o: orig.c foo_wrapper.o Makefile
	gcc -O3 -fPIC -c orig.c -o orig1.o
	objcopy --weaken-symbol=foo orig1.o orig2.o
	objcopy --redefine-sym foo=__real_foo orig1.o orig3.o
	ld -r --allow-multiple-definition orig2.o orig3.o -o orig.o

foo_wrapper.o: foo_wrapper.c Makefile
	gcc -O3 -c $<

clean:
	rm -f *.o a.out

dist: clean
	dir=`basename $$PWD` && cd .. && tar zcvf $$dir.tar.gz ./$$dir
	dir=`basename $$PWD` && ls -l ../$$dir.tar.gz
