run: a.out
	@ echo ""
	@ echo =====================================
	@ echo ==========BEGIN EXECUTION============
	@ echo =====================================
	./a.out

a.out: orig.c foo_wrapper.o
	gcc -c orig.c
	objcopy --redefine-sym foo=__wrapper_foo orig.o orig2.o
	objcopy --weaken-symbol=__wrapper_foo orig2.o orig3.o
	# objcopy --prefix-symbols=__real_ orig.o orig4.o
	gcc -Wl,--allow-multiple-definition orig3.o orig.o foo_wrapper.o -o a.out

foo_wrapper.o: foo_wrapper.c
	gcc -c $<

clean:
	rm -f *.o a.out

dist: clean
	dir=`basename $$PWD` && cd .. && tar zcvf $$dir.tar.gz ./$$dir
	dir=`basename $$PWD` && ls -l ../$$dir.tar.gz
