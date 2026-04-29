## New pragma directive

The `#pragma` directive can be used to influence compiler- and/or machine-specific settings inside a program. The new directives defined in TinyCore

```
#pragma arduino release_flags ...
#pragma arduino debug_flags ...
```

can be used for fine-grained optimization settings and to declare compile-time constants for all files of a project globally. Compiler flags mentioned in such a `#pragma` directive will be added to compile and link command lines. If the switch `Optimize for debugging` in the `Sketch` menu is not activated, then the compiler flags from the first type of directive above will be used. Otherwise, the flags mentioned after `debug_flags` will be used.

For example, if you want to make use of the `-mrelax` optimization that replaces `call` and `jmp` instructions with their shorter relative variants, `rcall` and `rjmp`, only if you are not debugging, you can do that as follows:

```
#pragma arduino release_flags -mrelax
```

Since this optimization is known to confuse the debugger, one should not use it when debugging. With 

```
#pragma arduino debug_flags -flto -DTXBUFFER
```

you will enable link-time optimization while debugging (`-flto`), which is normally disabled when debugging. Additionally, the symbol `TXBUFFER` will be defined, which enables a 16-byte transmit buffer for hardware serial interfaces.

