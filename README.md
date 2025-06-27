# Single-Cycle MIPS Processor

A complete implementation of a single-cycle MIPS processor in Verilog HDL, designed to execute a subset of the core MIPS instruction set architecture. This project demonstrates advanced digital design principles and computer architecture concepts through gate-level and RTL implementation.

## Overview

This project implements a fully functional single-cycle MIPS processor capable of executing meaningful programs. The processor is designed with modular components at the gate and RTL level, providing a comprehensive understanding of processor design and implementation. Each instruction is executed in a single clock cycle, following the architecture principles outlined in Patterson & Hennessy's "Computer Organization & Design."

## Instruction Set Architecture

The processor supports a comprehensive set of MIPS instructions across multiple categories:

**Memory Reference Instructions:**
- `lw` (Load Word) - Loads data from memory into a register
- `sw` (Store Word) - Stores register data to memory

**Arithmetic-Logical Instructions:**
- `add` - Performs addition of two registers
- `sub` - Performs subtraction of two registers  
- `and` - Performs bitwise AND operation
- `or` - Performs bitwise OR operation
- `slt` (Set Less Than) - Compares two registers and sets result

**Control Transfer Instructions:**
- `beq` (Branch Equal) - Conditional branch based on register equality
- `j` (Jump) - Unconditional jump to specified address

**Subroutine Support:**
- `jal` (Jump and Link) - Jump with return address storage for subroutine calls

## Architecture Design

The single-cycle processor architecture integrates the following key components:

**Core Processing Units:**
- Program Counter (PC) for instruction address management
- Instruction Memory for program storage
- Register File with 32 general-purpose 32-bit registers
- Arithmetic Logic Unit (ALU) for computational operations
- Data Memory for load/store operations

**Control Systems:**
- Main Control Unit for instruction decoding and control signal generation
- ALU Control Unit for operation-specific ALU configuration
- Sign Extension Unit for immediate value processing

**Supporting Logic:**
- Multiplexers for datapath selection and routing
- Shift units for address calculation in branch and jump operations
- Gate-level implementation of fundamental logic operations

## Technical Implementation

The processor follows a complete instruction execution cycle:

1. **Instruction Fetch:** Retrieves instruction from memory using PC address
2. **Instruction Decode:** Decodes opcode and generates control signals
3. **Execute:** Performs ALU operations or address calculations
4. **Memory Access:** Handles load/store operations with data memory
5. **Write Back:** Updates register file with computation results

The implementation emphasizes gate-level design principles, avoiding behavioral abstractions to provide deeper understanding of hardware functionality. All components are modularly designed for extensibility and educational clarity.

## Verification and Testing

Comprehensive simulation testing validates:
- Correct instruction fetch and execution for all supported operations
- Proper register file read/write operations
- Accurate memory access for load/store instructions
- Correct program counter updates for branch and jump instructions
- Verified ALU operation results across all arithmetic and logical functions

## Technical Significance

This implementation serves as a foundation for understanding advanced processor design concepts and demonstrates practical application of digital design principles. The single-cycle architecture provides clear insight into processor operation while maintaining the potential for extension to more complex implementations such as pipelined or superscalar designs.
