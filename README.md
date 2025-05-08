# `Minimalist binary crate for ASPEED

> Based on the  template for building applications for ARM Cortex-M microcontrollers


## Dependencies

To build embedded programs using this template you'll need:

- Rust  toolchain. 

- `rust-std` components (pre-compiled `core` crate) for the ARM Cortex-M
  targets. Run:

``` console
$ rustup target add thumbv7em-none-eabihf
```

## Building this app

$ cargo build --release

## Using this app

1. **Start the JLinkGDBServer**:
    ```sh
    JLinkGDBServer -device cortex-m4 -if swd
    ```

2. **Run the program with GDB**:
    ```sh
    gdb-multiarch target/thumbv7em-none-eabihf/release/aspeed-ddk
    
    ```

3. **Enable semihosting in GDB**:
    ```gdb
    target remote :2331
    monitor semihosting IOClient 2
    load
    continue
    ```