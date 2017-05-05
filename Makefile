all:
	gdbus-codegen --interface-prefix local. \
		--generate-c-code pms7003-generated \
		local.pms7003.xml
	@gcc -g3 pms7003.c pms7003-generated.c \
		-o pms7003 `pkg-config --cflags --libs gio-2.0 gio-unix-2.0`
clean:
	@rm -rf pms7003

.PHONY: clean all
