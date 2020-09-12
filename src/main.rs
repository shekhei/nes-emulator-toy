// +------------------------------------------------------------------------
// |
// |      MCS6502 MICROPROCESSOR INSTRUCTION SET - ALPHABETIC SEQUENCE
// |
// +------------------------------------------------------------------------
// |
// |     ADC   Add Memory to Accumulator with Carry
// |     AND   "AND" Memory with Accumulator
// |     ASL   Shift Left One Bit (Memory or Accumulator)
// |
// |     BCC   Branch on Carry Clear
// |     BCS   Branch on Carry Set
// |     BEQ   Branch on Result Zero
// |     BIT   Test Bits in Memory with Accumulator
// |     BMI   Branch on Result Minus
// |     BNE   Branch on Result not Zero
// |     BPL   Branch on Result Plus
// |     BRK   Force Break
// |     BVC   Branch on Overflow Clear
// |     BVS   Branch on Overflow Set
// |
// |     CLC   Clear Carry Flag
// |     CLD   Clear Decimal Mode
// |     CLI   Clear interrupt Disable Bit
// |     CLV   Clear Overflow Flag
// |     CMP   Compare Memory and Accumulator
// |     CPX   Compare Memory and Index X
// |     CPY   Compare Memory and Index Y
// |
// |     DEC   Decrement Memory by One
// |     dex   Decrement Index X by One
// |     DEY   Decrement Index Y by One
// |
// |     EOR   "Exclusive-Or" Memory with Accumulator
// |
// |     INC   Increment Memory by One
// |     INX   Increment Index X by One
// |     INY   Increment Index Y by One
// |
// |     JMP   Jump to New Location
// |
// +------------------------------------------------------------------------
//
//
// ------------------------------------------------------------------------+
//                                                                         |
//        MCS6502 MICROPROCESSOR INSTRUCTION SET - ALPHABETIC SEQUENCE     |
//                                                                         |
// ------------------------------------------------------------------------+
//                                                                         |
//       JSR   Jump to New Location Saving Return Address                  |
//                                                                         |
//       LDA   Load Accumulator with Memory                                |
//       LDX   Load Index X with Memory                                    |
//       LDY   Load Index Y with Memory                                    |
//       LSR   Shift Right One Bit (Memory or Accumulator)                 |
//                                                                         |
//       NOP   No Operation                                                |
//                                                                         |
//       ORA   "OR" Memory with Accumulator                                |
//                                                                         |
//       PHA   Push Accumulator on Stack                                   |
//       PHP   Push Processor Status on Stack                              |
//       PLA   Pull Accumulator from Stack                                 |
//       PLP   Pull Processor Status from Stack                            |
//                                                                         |
//       ROL   Rotate One Bit Left (Memory or Accumulator)                 |
//       ROR   Rotate One Bit Right (Memory or Accumulator)                |
//       RTI   Return from Interrupt                                       |
//       RTS   Return from Subroutine                                      |
//                                                                         |
//       SBC   Subtract Memory from Accumulator with Borrow                |
//       SEC   Set Carry Flag                                              |
//       SED   Set Decimal Mode                                            |
//       SEI   Set Interrupt Disable Status                                |
//       STA   Store Accumulator in Memory                                 |
//       STX   Store Index X in Memory                                     |
//       STY   Store Index Y in Memory                                     |
//                                                                         |
//       TAX   Transfer Accumulator to Index X                             |
//       TAY   Transfer Accumulator to Index Y                             |
//       TSX   Transfer Stack Pointer to Index X                           |
//       TXA   Transfer Index X to Accumulator                             |
//       TXS   Transfer Index X to Stack Pointer                           |
//       TYA   Transfer Index Y to Accumulator                             |
// ------------------------------------------------------------------------+

use std::env;
use std::error::Error;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;
use std::vec::Vec;
use std::rc::Weak;
use std::rc::Rc;
use std::cell::RefCell;
use std::cell::Ref;
mod ppu;
use ppu::PPU;

#[macro_use]
extern crate slog;
extern crate slog_term;
use slog::Logger;

const OPCODECYCLE : [u8; 16*16] = [
    7, 6, 0, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 0, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 0, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 0, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
    2, 6, 0, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
    2, 5, 0, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
];
const INESHEAD : [u8;4] = [0x4e,0x45,0x53,0x1a];
// const READBLOCK : usize = 0x400;

//    Bit0 - C - Carry flag: this holds the Carry out of the most significant
//    bit in any arithmetic operation. In subtraction operations however, this
//    flag is cleared - set to 0 - if a borrow is required, set to 1 - if no
//    borrow is required. The Carry flag is also used in shift and rotate
//    logical operations.
   //
//    Bit1 - Z - Zero flag: this is set to 1 when any arithmetic or logical
//    operation produces a zero result, and is set to 0 if the result is
//    non-zero.
   //
//    Bit 2 - I: this is an interrupt enable/disable flag. If it is set,
//    interrupts are disabled. If it is cleared, interrupts are enabled.
   //
//    Bit 3 - D: this is the decimal mode status flag. When set, and an Add with
//    Carry or Subtract with Carry instruction is executed, the source values are
//    treated as valid BCD (Binary Coded Decimal, eg. 0x00-0x99 = 0-99) numbers.
//    The result generated is also a BCD number.
   //
//    Bit 4 - B: this is set when a software interrupt (BRK instruction) is
//    executed.
   //
//    Bit 5: not used. Supposed to be logical 1 at all times.
   //
//    Bit 6 - V - Overflow flag: when an arithmetic operation produces a result
//    too large to be represented in a byte, V is set.
   //
//    Bit 7 - S - Sign flag: this is set if the result of an operation is
//    negative, cleared if positive.

struct Machine {
    ram: RefCell<[u8; 0x800]>,
    rom: Rc<RefCell<Vec<Op>>>,
    ppu: Rc<RefCell<PPU<Bank<u8>, glium::texture::Texture2d>>>,
    cpu: RefCell<CPU>,
}

use std::ops::{Index, IndexMut};

impl ppu::Write for glium::texture::Texture2d {
    fn save_b(&mut self, addr: u16, val: u8) {

    }
}

struct Bank<T> {
    data: Vec<T>,
    current_slot: usize,
    size: usize,
}

impl<T> Bank<T> {
    pub fn new(slots: usize, slot_size: usize) -> Bank<T> {
        Bank::<T> {
            data: Vec::with_capacity(slots*slot_size),
            current_slot:0,
            size: slot_size,
        }
    }
    pub fn change_slot(&mut self, slot: usize) {
        self.current_slot = slot;
    }
}

impl ppu::Read for Bank<u8>
{
    fn load_b(&mut self, addr: u16) -> u8 {
        let slot = self.current_slot;
        let size = self.size;
        let index = (((slot * size) as u16) + addr) as usize;
        if self.data.len() <= index {
            return 0
        }
        self.data[index]
    }
}

impl ppu::Write for Bank<u8>
{
    fn save_b(&mut self, addr: u16, val: u8) {
        let slot = self.current_slot;
        let size = self.size;
        let index = (((slot * size) as u16) + addr) as usize;
        if self.data.len() <= index {
            self.data.resize(index+1, 0u8);
        }
        self.data[index] = val;
    }
}

impl Machine {
    pub fn load(&self, pos: u16) -> u8 {
        if pos <= 0x2000 {
            // println!("loading {:x} = {:x}", pos, self.ram.borrow()[ pos as usize & 0x7ff ]);
            // tricks stolen from other emulators :P
            return self.ram.borrow()[ pos as usize & 0x7ff ]
        } else if pos <= 0x3fff {

            //PPU stuff, TODO
            let offset = (pos - 0x2000) % 8;
            return self.ppu.borrow_mut().load(offset+0x2000);
        }
        self.rom.borrow()[pos as usize - 0xc000].byte()
    }
    pub fn loadw(&self, pos: u16) -> u16 {
        self.load(pos) as u16 | ((self.load(pos+1) as u16) << 8)
    }
    pub fn loadw_wrap_addr(&self, pos: u16) -> u16 {
        self.load(pos) as u16 | ((self.load(((pos+1) as u8) as u16) as u16) << 8)
    }

    pub fn save(&self, pos: u16, val: u8) {
        if pos <= 0x2000 {
            // println!("saving {:x} = {:x}", pos, val);
            // tricks stolen from other emulators :P
            self.ram.borrow_mut()[ pos as usize & 0x7ff ] = val;
        } else if pos <= 0x3fff {
            self.ppu.borrow_mut().save(((pos-0x2000)%8)+0x2000, val);
            return
        } else if pos == 0x4014 {
            // special case, copy memory address over
            let val = ((val as u16) << 8) as usize;
            let (start, end) = (val as usize, val as usize +256usize);

            let slice = &self.ram.borrow()[start..end];
            self.ppu.borrow_mut().get_mut_oam_ref()[..].copy_from_slice(slice);
        }
    }
}
use std::fmt;

struct CPU {
    s: u8,
    pc: u16,
    a: u8,     // acc
    x: u8,       // xregister
    y: u8,       // yregister
    p: u8,   // S V - B D I Z C
    next_cycle: u16,
    machine: Weak<Machine>,
}

impl fmt::Debug for CPU {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "A:{:x} X:{:x} Y:{:x} P:{:x} SP:{:x} PC:{:x}", self.a, self.x, self.y, self.p, self.s, self.pc)
    }
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
enum OpType {
    ora, and, eor, adc, sbc, cmp, cpx, cpy, dec, dex, dey, inc, inx, iny, asl, rol, lsr, ror, lda,
    sta, ldx, stx, kil, ldy, sty, tax, txa, tay, tya, tsx, txs, pla, pha, plp, php, bpl, bmi, bvc,
    bvs, bcc, bcs, bne, beq, brk, rti, jsr, rts, jmp, jmpi, bit, clc, sec, cld, sed, cli, sei, clv, nop,
    xaa, lax, alr, anc, arr, axs, isc, dcp, sre, sax, rra, rla, slo, las, tas, shy, shx, ahx,
}

macro_rules! replace {
    ($vars:ty, $e:pat) => { $e }
}

macro_rules! defineOps {
    ( $( $name:ident $(, $vars:ty )* );* ) => {
        #[derive(Debug, Copy, Clone)]
        enum Op{
            $(
                $name( OpType, u8 $(, $vars )* ),
            )*
        }

        impl Op {
            pub fn byte(&self) -> u8 {
                use Op::*;
                match *self {
                    $(
                        $name( _, byte $(, replace!($vars, _ ) )* ) => byte,
                    )*
                }
            }
        }
    }
}

defineOps!(
    Implied;
    IndirectX, u8;
    IndirectY, u8;
    Immediate, u8;
    Zeropage, u8;
    ZeropageX, u8;
    ZeropageY, u8;
    Absolute, u16;        // basically zeropage or absolute, its the same, except zeropage has higherbyte ignored
    AbsoluteX, u16;
    AbsoluteY, u16;
    Relative, i8
);

#[derive(Debug, Copy, Clone)]
enum StatusFlags {
    Carry = 1,
    Zero = 1 << 1,
    Interrupt = 1 << 2,
    Decimal = 1 << 3,
    Break = 1 << 4,
    Overflow = 1 << 6,
    Negative = 1 << 7,
}

// #[derive(Debug)]
// enum Op {
//     Implied(OpType, u8),
//     IndirectX(OpType, u8, u8),
//     IndirectY(OpType, u8, u8),
//     Immediate(OpType, u8, u8),    // value is provided in the operand
//     Absolute(OpType, u8, u16),        // basically zeropage or absolute, its the same, except zeropage has higherbyte ignored
//     AbsoluteX(OpType, u8, u16),
//     AbsoluteY(OpType, u8, u16),
//     Indirect(OpType, u8, u16),
//     Relative(OpType, u8, u16),
// }

// impl Op {
//     pub fn byte(self: &Self) -> u8 {
//         use Op::*;
//         match *self {
//             Implied(_, byte) => byte,
//             IndirectX(_, byte, _) => byte,
//             IndirectY(_, byte, _) => byte,
//             Immediate(_, byte, _) => byte,
//             Absolute(_, byte, _) => byte,
//             AbsoluteX(_, byte, _) => byte,
//             AbsoluteY(_, byte, _) => byte,
//             Indirect(_, byte, _) => byte,
//             Relative(_, byte, _) => byte,
//         }
//     }
// }

// impl CPU {
//     pub fn new() -> CPU {
//         CPU {
//             a:0,
//             x:0,
//             y:0,
//             s:0xff,
//             p: 0,
//             pc:0,
//             machine: Rc<Machine>::new(),
//         }
//     }
// }

macro_rules! opmode {
    ( $opmode:ident, $byte:expr, $op:ident, ) => ( Op::$opmode(OpType::$op, $byte) );
    ( $opmode:ident, $byte:expr, $op:ident, $operand:expr ) => ( Op::$opmode(OpType::$op, $byte, $operand) );
}

macro_rules! op {
    ($comp:ident; _ => $others:block) => (
        $others
    );
    ($comp:ident; $( $opmode:ident: [ $( $hex:expr => $op:ident ),* ] ),* ; _ => $others:block) => (
        match $comp {
            $( $(
              $hex => opmode!($opmode, $hex, $op, ),
            )* )*
            _ => $others,
        }
    );
    ($comp:ident; $( $opmode:ident: $operand:expr; [ $( $hex:expr => $op:ident ),* ] ),* ; _ => $others:block) => (
        match $comp {
            $( $(
              $hex => opmode!($opmode, $hex, $op, $operand),
            )* )*
            _ => $others,
        }
    );
}

fn parse_op(buf: &mut [u8], i: usize) -> Option<Op> {
    match buf.get(i) {
        Some(v) => {
            let op = *v;
            // println!("op: {:x}", op);
            Some(op!(op;
                    Implied: [
                        0xca => dex, 0x88 => dey, 0xe8 => inx, 0xc8 => iny,
                        0x0a => asl, 0x2a => rol, 0x4a => lsr, 0x6a => ror,
                        0xaa => tax, 0x8a => txa, 0xa8 => tay, 0x98 => tya,
                        0xba => tsx, 0x9a => txs, 0x68 => pla, 0x48 => pha,
                        0x28 => plp, 0x08 => php, 0x00 => brk, 0x40 => rti,
                        0x60 => rts, 0x18 => clc, 0x38 => sec, 0xd8 => cld,
                        0xf8 => sed, 0x58 => cli, 0x78 => sei, 0xb8 => clv, 0xea => nop,
                        0x02 => kil, 0x12 => kil, 0x22 => kil, 0x32 => kil, 0x42 => kil,
                        0x52 => kil, 0x62 => kil, 0x72 => kil, 0x92 => kil, 0xb2 => kil,
                        0xd2 => kil, 0xf2 => kil,
                        0x1a => nop, 0x3a => nop, 0x5a => nop, 0x7a => nop, 0xda => nop, 0xfa => nop
                    ];
                    _ => {
                        let oper = match buf.get(i+1) {
                            Some(x) => *x,
                            None => return None,
                        };
                        // println!("oper lb: {:x}", oper);
                        op!(
                            op;
                            Immediate: oper; [
                                0x09 => ora, 0x29 => and, 0x49 => eor, 0x69 => adc, 0xe9 => sbc, 0xc9 => cmp,
                                0xe0 => cpx, 0xc0 => cpy, 0xa9 => lda, 0xa2 => ldx, 0xa0 => ldy, 0xab => lax,
                                0x0b => anc, 0x2b => anc, 0x4b => alr, 0x6b => arr, 0x8b => xaa, 0xcb => axs,
                                0xeb => sbc,
                                0x80 => nop, 0x82 => nop, 0xc2 => nop, 0xe2 => nop, 0x89 => nop
                            ],
                            Zeropage: oper; [
                                0x05 => ora, 0x25 => and, 0x45 => eor, 0x65 => adc, 0xe5 => sbc, 0xc5 => cmp,
                                0xe4 => cpx, 0xc4 => cpy, 0xc6 => dec, 0xe6 => inc, 0x06 => asl, 0x26 => rol,
                                0x46 => lsr, 0x66 => ror, 0xa5 => lda, 0x85 => sta, 0xa6 => ldx, 0x86 => stx,
                                0xa4 => ldy, 0x84 => sty, 0x24 => bit,
                                0x07 => slo, 0x27 => rla, 0x47 => sre, 0x67 => rra, 0x87 => sax, 0xa7 => lax,
                                0xc7 => dcp, 0xe7 => isc,
                                0x04 => nop, 0x44 => nop, 0x64 => nop
                            ],
                            ZeropageX: oper; [
                                0x15 => ora, 0x35 => and, 0x55 => eor, 0x75 => adc, 0xf5 => sbc, 0xd5 => cmp,
                                0xd6 => dec, 0xf6 => inc, 0x16 => asl, 0x36 => rol, 0x56 => lsr, 0x76 => ror,
                                0xb5 => lda, 0x95 => sta, 0xb4 => ldy, 0x94 => sty,
                                0x17 => slo, 0x37 => rla, 0x57 => sre, 0x77 => rra, 0xd7 => dcp, 0xf7 => isc,
                                0x14 => nop, 0x34 => nop, 0x54 => nop,
                                0x74 => nop, 0xd4 => nop, 0xf4 => nop
                            ],
                            ZeropageY: oper; [
                                0xb6 => ldx, 0x96 => stx, 0x97 => sax, 0xb7 => lax
                            ],
                            IndirectX: oper; [
                                0x01 => ora, 0x21 => and, 0x41 => eor, 0x61 => adc, 0xe1 => sbc, 0xc1 => cmp,
                                0xa1 => lda, 0x81 => sta,
                                0x03 => slo, 0x23 => rla, 0x43 => sre, 0x63 => rra, 0x83 => sax, 0xa3 => lax,
                                0xc3 => dcp, 0xe3 => isc
                            ],
                            Relative: oper as i8; [
                                0x10 => bpl, 0x30 => bmi, 0x50 => bvc, 0x70 => bvs, 0x90 => bcc, 0xb0 => bcs,
                                0xd0 => bne, 0xf0 => beq
                            ],
                            IndirectY: oper; [
                                0x11 => ora, 0x31 => and, 0x51 => eor, 0x71 => adc, 0xf1 => sbc, 0xd1 => cmp,
                                0xb1 => lda, 0x91 => sta,
                                0x13 => slo, 0x33 => rla, 0x53 => sre, 0x73 => rra, 0xb3 => lax,
                                0xd3 => dcp, 0xf3 => isc, 0x93 => ahx
                            ];
                            _ => {

                                let mut dwoper = match buf.get(i+2) {
                                    Some(x) => *x,
                                    None => {
                                        return None
                                    }
                                } as u16;
                                dwoper <<= 8;
                                dwoper += oper as u16;
                                // println!("oper w/hb: {:x}", dwoper);
                                op!(
                                    op;
                                    Absolute: dwoper; [
                                        0x0d => ora, 0x2d => and, 0x4d => eor, 0x6d => adc, 0xed => sbc, 0xcd => cmp,
                                        0xec => cpx, 0xcc => cpy, 0xce => dec, 0xee => inc, 0x0e => asl, 0x2e => rol,
                                        0x4e => lsr, 0x6e => ror, 0xad => lda, 0x8d => sta, 0xae => ldx, 0x8e => stx,
                                        0xac => ldy, 0x8c => sty, 0x20 => jsr, 0x4c => jmp, 0x6c => jmpi, 0x2c => bit,
                                        0x0f => slo, 0x2f => rla, 0x4f => sre, 0x6f => rra, 0x8f => sax, 0xaf => lax,
                                        0xcf => dcp, 0xef => isc,
                                        0x0c => nop
                                    ],
                                    AbsoluteX: dwoper; [
                                        0x1d => ora, 0x3d => and, 0x5d => eor, 0x7d => adc, 0xfd => sbc, 0xdd => cmp,
                                        0xde => dec, 0xfe => inc, 0x1e => asl, 0x3e => rol, 0x5e => lsr, 0x7e => ror,
                                        0xbd => lda, 0x9d => sta, 0xbc => ldy,
                                        0x1f => slo, 0x3f => rla, 0x5f => sre, 0x7f => rra,
                                        0xdf => dcp, 0xff => isc, 0x9c => shy,
                                        0x1c => nop, 0x3c => nop, 0x5c => nop,
                                        0x7c => nop, 0xdc => nop, 0xfc => nop
                                    ],
                                    AbsoluteY: dwoper; [
                                        0x19 => ora, 0x39 => and, 0x59 => eor, 0x79 => adc, 0xf9 => sbc, 0xd9 => cmp,
                                        0xb9 => lda, 0x99 => sta, 0xbe => ldx,
                                        0x1b => slo, 0x3b => rla, 0x5b => sre, 0x7b => rra, 0xbf => lax,
                                        0xdb => dcp, 0xfb => isc, 0x9f => ahx, 0x9e => shx, 0x9b => tas, 0xbb => las
                                    ];
                                    _ => { panic!("Unknown instruction {:x}, at {}", op, i) }
                                )
                            }
                        )
                }))
        },
        None => None
    }
}

trait AM {
    fn save(cpu: &mut CPU, val: u8, oper: u16);
    fn load(cpu: &mut CPU, oper: u16) -> u8;
}

struct MemAM;
impl AM for MemAM {
    fn save(cpu: &mut CPU, val: u8, oper: u16 ) {
        match cpu.machine.upgrade() {
            Some(machine) => machine.save(oper, val),
            None => unreachable!(),
        };
    }
    fn load(cpu: &mut CPU, oper: u16) -> u8 {
        match cpu.machine.upgrade() {
            Some(machine) => machine.load(oper),
            None => unreachable!(),
        }
    }
}
struct AccAM;
impl AM for AccAM {
    fn save(cpu: &mut CPU, val: u8, _: u16) {
        cpu.a = cpu.set_zn_n_ret(val);
    }
    fn load(cpu: &mut CPU, oper: u16) -> u8 {
        cpu.a
    }
}

struct ImmAM;
impl AM for ImmAM {
    fn save(_: &mut CPU, val: u8, _: u16) {
        panic!("cant be used");
    }
    fn load(_: &mut CPU, oper: u16) -> u8 {
        oper as u8
    }
}


fn ines(file: &mut File, display: &glium::Display, texture: Rc<RefCell<glium::texture::Texture2d>>) -> Rc<Machine> {
    // we have reached iNES, the header is yet to be parsed
    // first 4 bytes are already read
    // 0, rom bank count
    // 1, vrom bank count
    // the rest is not important for now

    let mut header = [0u8; 12];
    match file.read_exact(&mut header) {
        Err(why) => panic!("File is corrupted, cannot even finish reading header: {}", why.description()),
        Ok(_) => (),
    }

    // lets first read the first bank into memory and then run one by one
    // lets first use a vector, improve in teh future
    let rom_count = header[0];
    let rom = Rc::new(RefCell::new(Vec::new()));

    let vrom_count = header[1] as usize;
    info!("## vrom count {}", vrom_count);

    let rom_bank = Rc::new(RefCell::new(Bank::<u8>::new(vrom_count, 0x2000)));
    let vram = ppu::Vram::new(rom_bank.clone());

    let ppu = Rc::new(RefCell::new(PPU::new(vram, texture.clone())));
    let machine = Machine{ram: RefCell::new([0u8; 0x800]), rom: rom.clone(), ppu: ppu.clone(), cpu: RefCell::new(CPU{next_cycle: 0,a:0, x:0, y:0, p:0b00100100, s:0xfd, pc:0, machine: Weak::new()}) };

    {
        let mut rom = machine.rom.borrow_mut();
        rom.reserve(0x4000);
        // lets do a simple parse and read in blocks of 1k
        let mut buf = [0u8; 0x4000];
        let n = match file.read_exact(&mut buf[..]) {
            Err(why) => panic!("Error reading data: {}", why.description()),
            Ok(_) => 0x4000,
        };
        let mut i = 0;
        while i < n {
            match parse_op(&mut buf[..], i) {
                Some(x) => {
                    rom.push(x)
                },
                None => {
                    break
                },
            }
            i+=1
        }
    }
    info!("total ops {}", rom.borrow().len());



    // READ VROM
    match file.read_exact(&mut rom_bank.borrow_mut().data[..]) {
        Err(why) => panic!("Failed while reading chr-rom: {}", why.description()),
        Ok(_) => (),
    }
    // all memory offsets 0x8000
    let reset: u16 = ( (machine.load(0xfffd) as u16) << 8 ) + machine.load(0xfffc) as u16;
    machine.cpu.borrow_mut().pc = reset;
    // machine.cpu.borrow_mut().pc = 0xc000;
    info!("let us begin: {:x}", reset);
    let machineRc = Rc::new(machine);
    machineRc.cpu.borrow_mut().machine = Rc::downgrade(&machineRc);
    return machineRc;
}
use std::ops::Deref;

impl Machine {
    pub fn exec(machine: Rc<Machine>, cycle: u16) {
        use OpType::*;
        use Op::*;
        let op : Op;

        if cycle == machine.cpu.borrow().next_cycle {
            {
                let mut cpu = machine.cpu.borrow_mut();
                let rom = machine.rom.borrow_mut();
                if cpu.pc > 0xc000 {
                    op = rom[(cpu.pc as usize) % 0xc000];
                } else {
                    let mut opcodes = [0u8;3];
                    let pc = cpu.pc;
                    opcodes[0] = machine.load(pc);
                    opcodes[1] = machine.load(pc+1);
                    opcodes[2] = machine.load(pc+2);
                    op = match parse_op(&mut opcodes[..], 0) {
                        Some(x) => x,
                        None => unimplemented!(),
                    };
                }
                // debug!("{:x} {:?} {:?} {}", cpu.borrow().pc, op, cpu.borrow(), cycle);
                cpu.pc += 1; // incrementing by single byte
                cpu.next_cycle.wrapping_add(OPCODECYCLE[op.byte() as usize] as u16);
            }
            match op {
                Implied(ref optype, _) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    match *optype {
                        dex => cpu.dex(),
                        dey => cpu.dey(),
                        inx => cpu.inx(),
                        iny => cpu.iny(),
                        asl => cpu.asl::<AccAM>(0),
                        rol => cpu.rol::<AccAM>(0),
                        lsr => cpu.lsr::<AccAM>(0),
                        ror => cpu.ror::<AccAM>(0),
                        tax => cpu.tax(),
                        txa => cpu.txa(),
                        tay => cpu.tay(),
                        tya => cpu.tya(),
                        tsx => cpu.tsx(),
                        txs => cpu.txs(),
                        pla => cpu.pla(),
                        pha => cpu.pha(),
                        plp => cpu.plp(),
                        php => cpu.php(),
                        brk => cpu.brk(),
                        rti => cpu.rti(),
                        rts => cpu.rts(),
                        clc => cpu.clc(),
                        sec => cpu.sec(),
                        cld => cpu.cld(),
                        sed => cpu.sed(),
                        cli => cpu.cli(),
                        sei => cpu.sei(),
                        clv => cpu.clv(),
                        nop => cpu.nop(),
                        _ => panic!("implement {:?}", op)
                    }
                },
                Immediate(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    cpu.pc += 1;
                    let oper = oper as u16;

                    match *optype {
                        lda => cpu.lda::<ImmAM>(oper),
                        ldx => cpu.ldx::<ImmAM>(oper),
                        ldy => cpu.ldy::<ImmAM>(oper),
                        and => cpu.and::<ImmAM>(oper),
                        cmp => cpu.cmp::<ImmAM>(oper),
                        cpy => cpu.cpy::<ImmAM>(oper),
                        cpx => cpu.cpx::<ImmAM>(oper),
                        ora => cpu.ora::<ImmAM>(oper),
                        eor => cpu.eor::<ImmAM>(oper),
                        adc => cpu.adc::<ImmAM>(oper),
                        sbc => cpu.sbc::<ImmAM>(oper),
                        nop => cpu.nop(),
                        _ => panic!("implement {:?}", op)
                    }
                },
                AbsoluteX(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    let oper = oper.wrapping_add(cpu.x as u16);
                    cpu.pc += 2;
                    match *optype {
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        dec => cpu.dec::<MemAM>(oper),
                        inc => cpu.inc::<MemAM>(oper),
                        asl => cpu.asl::<MemAM>(oper),
                        rol => cpu.rol::<MemAM>(oper),
                        lsr => cpu.lsr::<MemAM>(oper),
                        ror => cpu.ror::<MemAM>(oper),
                        //move cmd
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        ldy => cpu.ldy::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                AbsoluteY(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    let oper = oper.wrapping_add(cpu.y as u16);
                    cpu.pc += 2;
                    match *optype {
                        //logical and arthemetic cmd
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        //move cmd
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        ldx => cpu.ldx::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                Absolute(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    // let val = machine.load(oper);
                    // let oper = oper as u16;
                    cpu.pc += 2;
                    match *optype {
                        //logical and arthemetic cmd
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        cpx => cpu.cpx::<MemAM>(oper),
                        cpy => cpu.cpy::<MemAM>(oper),
                        dec => cpu.dec::<MemAM>(oper),
                        inc => cpu.inc::<MemAM>(oper),
                        asl => cpu.asl::<MemAM>(oper),
                        rol => cpu.rol::<MemAM>(oper),
                        lsr => cpu.lsr::<MemAM>(oper),
                        ror => cpu.ror::<MemAM>(oper),
                        //move cmd
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        ldx => cpu.ldx::<MemAM>(oper),
                        stx => cpu.stx::<MemAM>(oper),
                        ldy => cpu.ldy::<MemAM>(oper),
                        sty => cpu.sty::<MemAM>(oper),
                        //jmp or flags
                        jsr => cpu.jsr::<MemAM>(oper),
                        jmp => cpu.jmp::<MemAM>(oper),
                        jmpi => cpu.jmpi::<MemAM>(oper),
                        bit => cpu.bit::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        sax => cpu.sax::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }

                },
                Zeropage(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    let oper = oper as u16;
                    cpu.pc += 1;
                    match *optype {
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        cpx => cpu.cpx::<MemAM>(oper),
                        cpy => cpu.cpy::<MemAM>(oper),
                        asl => cpu.asl::<MemAM>(oper),
                        rol => cpu.rol::<MemAM>(oper),
                        lsr => cpu.lsr::<MemAM>(oper),
                        ror => cpu.ror::<MemAM>(oper),
                        stx => cpu.stx::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        bit => cpu.bit::<MemAM>(oper),
                        lda => cpu.lda::<MemAM>(oper),
                        ldy => cpu.ldy::<MemAM>(oper),
                        ldx => cpu.ldx::<MemAM>(oper),
                        sty => cpu.sty::<MemAM>(oper),
                        dec => cpu.dec::<MemAM>(oper),
                        inc => cpu.inc::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        sax => cpu.sax::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                ZeropageX(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    let oper = oper.wrapping_add(cpu.x) as u16;
                    cpu.pc += 1;
                    match *optype {
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        dec => cpu.dec::<MemAM>(oper),
                        inc => cpu.inc::<MemAM>(oper),
                        asl => cpu.asl::<MemAM>(oper),
                        rol => cpu.rol::<MemAM>(oper),
                        lsr => cpu.lsr::<MemAM>(oper),
                        ror => cpu.ror::<MemAM>(oper),
                        //move cmd
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        ldy => cpu.ldy::<MemAM>(oper),
                        sty => cpu.sty::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                ZeropageY(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    let oper = oper.wrapping_add(cpu.y) as u16;
                    cpu.pc += 1;
                    match *optype {
                        // move cmd
                        ldx => cpu.ldx::<MemAM>(oper),
                        stx => cpu.stx::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        sax => cpu.sax::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                Relative(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    cpu.pc += 1;
                    match *optype {
                        bpl => cpu.bpl(oper),
                        bcs => cpu.bcs(oper),
                        bcc => cpu.bcc(oper),
                        beq => cpu.beq(oper),
                        bne => cpu.bne(oper),
                        bvs => cpu.bvs(oper),
                        bvc => cpu.bvc(oper),
                        bmi => cpu.bmi(oper),
                        nop => cpu.nop(),
                        _ => panic!("implement {:?}", op),
                    }

                },
                IndirectX(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    cpu.pc += 1;
                    let oper = cpu.x.wrapping_add(oper) as u16;
                    let oper = machine.loadw_wrap_addr(oper);
                    match *optype {
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        sax => cpu.sax::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
                IndirectY(ref optype, _, oper) => {
                    let mut cpu = machine.cpu.borrow_mut();
                    cpu.pc += 1;
                    let oper = machine.loadw_wrap_addr(oper as u16).wrapping_add(cpu.y as u16) as u16;
                    match *optype {
                        ora => cpu.ora::<MemAM>(oper),
                        and => cpu.and::<MemAM>(oper),
                        eor => cpu.eor::<MemAM>(oper),
                        adc => cpu.adc::<MemAM>(oper),
                        sbc => cpu.sbc::<MemAM>(oper),
                        cmp => cpu.cmp::<MemAM>(oper),
                        lda => cpu.lda::<MemAM>(oper),
                        sta => cpu.sta::<MemAM>(oper),
                        nop => cpu.nop(),
                        // illegal ops
                        slo => cpu.slo::<MemAM>(oper),
                        rla => cpu.rla::<MemAM>(oper),
                        sre => cpu.sre::<MemAM>(oper),
                        rra => cpu.rra::<MemAM>(oper),
                        lax => cpu.lax::<MemAM>(oper),
                        dcp => cpu.dcp::<MemAM>(oper),
                        isc => cpu.isc::<MemAM>(oper),
                        _ => panic!("implement {:?}", op),
                    }
                },
            }
        }
        let machineT = machine;
        machineT.ppu.borrow_mut().tick();
    }
}

impl CPU {
    // illegal ops
    pub fn slo<T: AM>(&mut self, oper:u16) {
        self.asl::<T>(oper);
        self.ora::<T>(oper);
    }
    pub fn rla<T: AM>(&mut self, oper:u16) {
        self.rol::<T>(oper);
        self.and::<T>(oper);
    }
    pub fn sre<T: AM>(&mut self, oper:u16) {
        self.lsr::<T>(oper);
        self.eor::<T>(oper);
    }
    pub fn rra<T: AM>(&mut self, oper:u16) {
        self.ror::<T>(oper);
        self.adc::<T>(oper);
    }
    pub fn sax<T: AM>(&mut self, oper:u16) {
        let a = self.a;
        let x = self.x;
        T::save(self, a&x, oper);
    }
    pub fn lax<T: AM>(&mut self, oper:u16) {
        self.lda::<T>(oper);
        self.ldx::<T>(oper);
    }
    pub fn dcp<T: AM>(&mut self, oper:u16) {
        self.dec::<T>(oper);
        self.cmp::<T>(oper);
    }
    pub fn isc<T: AM>(&mut self, oper:u16) {
        self.inc::<T>(oper);
        self.sbc::<T>(oper);
    }
    //transfer
    pub fn tsx(&mut self) {
        let s = self.s;
        self.x = self.set_zn_n_ret(s);
    }
    pub fn txs(&mut self) {
        self.s = self.x;
    }
    pub fn tay(&mut self) {
        let a = self.a;
        self.y = self.set_zn_n_ret(a);
    }
    pub fn tya(&mut self) {
        let y = self.y;
        self.a = self.set_zn_n_ret(y);
    }
    pub fn txa(&mut self) {
        let x = self.x;
        self.a = self.set_zn_n_ret(x);
    }
    pub fn tax(&mut self) {
        let a = self.a;
        self.x = self.set_zn_n_ret(a);
    }
    pub fn tex(&mut self) {
        unimplemented!();
    }
    pub fn nop(&mut self) {
    }
    pub fn compare<T: AM>(&mut self, lhs: u8, rhs: u8) {
        let _ = self.set_zn_n_ret(lhs.wrapping_sub(rhs));
        self.set_flag(StatusFlags::Carry, lhs >= rhs);
    }
    pub fn cmp<T: AM>(&mut self, oper:u16) {
        let val = T::load(self, oper);
        let a = self.a;
        self.compare::<T>(a, val);
    }
    pub fn cpx<T: AM>(&mut self, oper:u16) {
        let val = T::load(self, oper);
        let x = self.x;
        self.compare::<T>(x, val);
    }
    pub fn cpy<T: AM>(&mut self, oper:u16) {
        let val = T::load(self, oper);
        let y = self.y;
        self.compare::<T>(y, val);
    }
    pub fn clv(&mut self) {
        self.set_flag(StatusFlags::Overflow, false);
    }
    pub fn cli(&mut self) {
        unimplemented!();
    }
    pub fn sed(&mut self) {
        self.set_flag(StatusFlags::Decimal, true);
    }
    pub fn cld(&mut self) {
        self.set_flag(StatusFlags::Decimal, false);
    }
    pub fn sec(&mut self) {
        self.set_flag(StatusFlags::Carry, true);
    }
    pub fn clc(&mut self) {
        self.set_flag(StatusFlags::Carry, false);
    }
    pub fn rts(&mut self) {
        self.pc = self.pop_w() + 1;
    }
    pub fn pop_b(&mut self) -> u8 {
        self.s += 1;
        let s = self.s as u16 + 0x100;
        return MemAM::load(self, s)
    }
    pub fn pop_w(&mut self) -> u16 {
        let a = self.pop_b() as u16;
        a + ((self.pop_b() as u16) << 8)
    }
    pub fn rti(&mut self) {
        let p = self.pop_b();
        self.set_p(p);
        self.pc = self.pop_w();
    }
    pub fn brk(&mut self) {
        unimplemented!();
    }
    pub fn php(&mut self) {
        let s = self.s as u16 + 0x100;
        let p = self.p;
        MemAM::save(self, p | (StatusFlags::Break as u8), s);
        self.s -= 1;
    }
    pub fn plp(&mut self) {
        self.s += 1;
        let s = self.s as u16 + 0x100;
        let p = MemAM::load(self, s);
        self.set_p(p);
    }
    pub fn pha(&mut self) {
        let a = self.a;
        let s = self.s as u16 + 0x100;
        MemAM::save(self, a, s);
        self.s -= 1;
    }
    pub fn pla(&mut self) {
        self.s += 1;
        let s = self.s as u16 + 0x100;
        let a = MemAM::load(self, s);
        self.a = self.set_zn_n_ret(a);
    }
    pub fn lsr<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        self.set_flag(StatusFlags::Carry, (val & 1) == 1);
        val >>= 1;
        T::save(self, val, oper);
    }
    pub fn ror<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        let carry = self.get_flag(StatusFlags::Carry);
        self.set_flag(StatusFlags::Carry, (val & 1) == 1);
        val >>= 1;
        if carry {
            val |= 0b10000000;
        }

        T::save(self, val, oper);
    }
    pub fn rol<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        let carry = self.get_flag(StatusFlags::Carry);
        self.set_flag(StatusFlags::Carry, (val & 0b10000000) == 0b10000000);
        val <<= 1;
        if carry {
            val += 1;
        }
        T::save(self, val, oper);
    }
    pub fn asl<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        self.set_flag(StatusFlags::Carry, (val & 0b10000000) == 0b10000000);
        val <<= 1;
        T::save(self, val, oper);
    }
    pub fn iny(&mut self) {
        let y = self.y;
        self.y = self.set_zn_n_ret(y.wrapping_add(1));
    }
    pub fn inx(&mut self) {
        let x = self.x;
        self.x = self.set_zn_n_ret(x.wrapping_add(1));
    }
    pub fn dec<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        val = self.set_zn_n_ret(val.wrapping_sub(1));
        T::save(self, val, oper);
    }
    pub fn inc<T: AM>(&mut self, oper: u16) {
        let mut val = T::load(self, oper);
        val = self.set_zn_n_ret(val.wrapping_add(1));
        T::save(self, val, oper);
    }
    // jump/branch ops
    pub fn jmp<T: AM>(&mut self, oper:u16) {
        self.pc = oper;
    }
    pub fn jmpi<T: AM>(&mut self, oper:u16) {
        // apparently there is a super crazy cpu bug
        let lo = T::load(self, oper);
        let hihi = oper & 0xff00;
        let hilo = (oper as u8).wrapping_add(1) as u16;

        // apparently the bug is to do with $C0ff next position is $C000 instead of $C100
        let hi = T::load(self, hihi + hilo);
        self.pc = (hi as u16) << 8 | lo as u16;
    }
    pub fn jsr<T: AM>(&mut self, oper: u16) {
        let pc = self.pc - 1;
        let s = self.s;
        T::save(self, (pc >> 8) as u8, s as u16 + 0x100);
        self.s -= 1;
        let s = self.s;
        T::save(self, (pc & 0b11111111) as u8, s as u16 + 0x100);
        self.s -= 1;
        self.pc = oper;
    }
    pub fn dex(&mut self) {
        let x = self.x;
        self.x = self.set_zn_n_ret(x.wrapping_sub(1));
    }
    pub fn dey(&mut self) {
        let y = self.y;
        self.y = self.set_zn_n_ret(y.wrapping_sub(1));
    }
    pub fn sei(&mut self) {
        self.set_flag(StatusFlags::Interrupt, true)
    }

    pub fn set_flag(&mut self, flag: StatusFlags, on: bool) {
        if on {
            self.p |= flag as u8;
        } else {
            self.p &= !(flag as u8);
        }
    }
    pub fn set_p(&mut self, p: u8) {
        self.p = (p | 0x30) - 0x10; // weird stuff.. basically setting flag 5 and flag 4 to 10
    }
    pub fn get_flag(&mut self, flag: StatusFlags) -> bool {
        return (self.p & (flag as u8)) == (flag as u8);
    }
    pub fn set_zn_n_ret(&mut self, oper: u8) -> u8 {
        // println!("Setting Zero Flag {}", oper == 0);
        // println!("Setting Negative Flag {}", (oper & 0x80) != 0);
        self.set_flag(StatusFlags::Zero, oper == 0);
        self.set_flag(StatusFlags::Negative, (oper & 0x80) != 0);
        oper
    }

    // load to register
    pub fn lda<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        self.a = self.set_zn_n_ret(val);
    }
    pub fn ldx<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        self.x = self.set_zn_n_ret(val);
    }
    pub fn ldy<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        self.y = self.set_zn_n_ret(val);
    }
    // save register to

    pub fn sta<T: AM>(&mut self, oper:u16) {
        let a = self.a;
        T::save(self, a, oper);
    }
    pub fn stx<T: AM>(&mut self, oper:u16) {
        let x = self.x;
        T::save(self, x, oper);
    }
    pub fn sty<T: AM>(&mut self, oper:u16) {
        let y = self.y;
        T::save(self, y, oper);
    }
    pub fn bit<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        let a = self.a;
        self.set_flag(StatusFlags::Zero, (val & a) == 0);
        self.set_flag(StatusFlags::Negative, ( val & 0x80 ) != 0 );
        // println!("{:b}, {}", val, ( val & (StatusFlags::Overflow as u8) >> 5 ));
        self.set_flag(StatusFlags::Overflow, ( val & 0x40 ) != 0 );
    }
    pub fn eor<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        let a = self.a ^ val;
        self.a = self.set_zn_n_ret(a);
    }
    pub fn and<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        let a = self.a & val;
        self.a = self.set_zn_n_ret(a);
    }
    pub fn ora<T: AM>(&mut self, oper: u16) {
        let val = T::load(self, oper);
        let a = self.a | val;
        self.a = self.set_zn_n_ret(a);
    }
    // branch
    pub fn bpl(&mut self, oper: i8) {
        if !self.get_flag(StatusFlags::Negative) {
            self.pc = ( self.pc as i32 + oper as i32 ) as u16;
        }
    }
    pub fn bmi(&mut self, oper: i8) {
        if self.get_flag(StatusFlags::Negative) {
            self.pc = ( self.pc as i32 + oper as i32 ) as u16;
        }
    }
    pub fn bcs(&mut self, oper: i8) {
        if self.get_flag(StatusFlags::Carry) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn bcc(&mut self, oper: i8) {
        if !self.get_flag(StatusFlags::Carry) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn bvs(&mut self, oper: i8) {
        if self.get_flag(StatusFlags::Overflow) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn bvc(&mut self, oper: i8) {
        if !self.get_flag(StatusFlags::Overflow) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn beq(&mut self, oper: i8) {
        if self.get_flag(StatusFlags::Zero) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn bne(&mut self, oper: i8) {
        if !self.get_flag(StatusFlags::Zero) {
            let mut pc = self.pc as i32;
            pc += oper as i32;
            self.pc = pc as u16;
        }
    }
    pub fn sbc<T: AM>(&mut self, oper:u16) {
        let val = 0xff - T::load(self, oper);
        let a = self.a;
        let mut result = a as u16 + val as u16;
        if self.get_flag(StatusFlags::Carry) {
            result += 1;
        }
        self.set_flag(StatusFlags::Carry, (result & 0x100) != 0);
        let result = result as u8;
        let a = self.a;
        self.set_flag(StatusFlags::Overflow, (a ^ val) & 0x80 == 0 && (a ^ result) & 0x80 == 0x80);
        self.a = self.set_zn_n_ret(result);
    }
    pub fn adc<T: AM>(&mut self, oper:u16) {
        let val = T::load(self, oper);
        let a = self.a;
        let mut result = a as u16 + val as u16;
        if self.get_flag(StatusFlags::Carry) {
            result += 1;
        }
        self.set_flag(StatusFlags::Carry, (result & 0x100) != 0);
        let result = result as u8;
        let a = self.a;
        self.set_flag(StatusFlags::Overflow, (a ^ val) & 0x80 == 0 && (a ^ result) & 0x80 == 0x80);
        self.a = self.set_zn_n_ret(result);
    }
}

fn start(path: &Path) {

    let display = path.display();
    println!("Using file {}", display);

    use glium::{DisplayBuild, Surface};
    let window = glium::glutin::WindowBuilder::new()
        .with_dimensions(256, 240)
        .build_glium()
        .unwrap();

    let mut file = match File::open(path) {
        Err(why) => panic!("Couldnt open {}: {}", display, why.description()),
        Ok(file) => file,
    };

    let mut take = [0u8;4];
    match file.read_exact(&mut take) {
        Err(why) => panic!("Couldnt read {} header: {}", display, why.description()),
        Ok(_) => (),
    };
    println!("header = \"{:?}\"", take);

    #[derive(Copy, Clone)]
    struct Vertex {
        position: [f32; 2],
        tex_coords: [f32; 2],
    }

    implement_vertex!(Vertex, position, tex_coords);


    let vb = glium::VertexBuffer::new(&window, &[
            Vertex { position: [-1.0,  -1.0], tex_coords: [0.0, 0.0] },
            Vertex { position: [-1.0,  1.0], tex_coords: [0.0, 1.0] },
            Vertex { position: [1.0, 1.0], tex_coords: [1.0, 1.0] },
            Vertex { position: [1.0, -1.0], tex_coords: [1.0, 0.0] },
        ]).unwrap();

    let ib = glium::IndexBuffer::new(&window, glium::index::PrimitiveType::TriangleStrip, &[1u8, 2, 0, 3]).unwrap();
    let texture = Rc::new(RefCell::new(glium::texture::Texture2d::empty(&window, 256, 240).unwrap()));
    texture.borrow().as_surface().clear_color(0.0,0.0,1.0,1.0);
    let machine = match take {
        INESHEAD => ines(&mut file, &window, texture.clone()),
        _ => panic!("File format not supported"),
    };

    let vertex_shader_src = r#"
        #version 140
        in vec2 position;
        // in vec3 normal;
        in vec2 tex_coords;
        out vec2 v_tex_coords;
        uniform mat4 matrix;
        void main() {
            v_tex_coords = tex_coords;
            gl_Position = matrix * vec4(position, 1.0, 1.0);
        }
    "#;

    let fragment_shader_src = r#"
        #version 140
        in vec2 v_tex_coords;
        uniform sampler2D tex;
        out vec4 color;

        void main() {
            color = texture(tex, v_tex_coords);
        }
    "#;

    let program = glium::Program::from_source(&window, vertex_shader_src, fragment_shader_src,
                                              None).unwrap();
    let mut cycle = 0u16;

    loop {
        // listing the events produced by the window and waiting to be received

        Machine::exec(machine.clone(), cycle);
        let mut target = window.draw();
        target.clear_color(0.0, 0.0, 0.0, 1.0);
        let tc = texture.clone();
        let t = tc.borrow();
        let uniforms = uniform! {
            matrix: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0f32]
            ],
            tex: &*t,
        };
        target.draw(&vb, &ib, &program, &uniforms,
                    &Default::default()).unwrap();
        // texture.borrow().as_surface().draw(&vb, &ib, &target, &uniform!{ texture: &$texture },
                                //  &Default::default()).unwrap();
        // texture.borrow().as_surface().fill(&target, glium::uniforms::MagnifySamplerFilter::Linear);
        target.finish().unwrap();
        cycle.wrapping_add(1);

        for ev in window.poll_events() {
            match ev {
                glium::glutin::Event::Closed => return,   // the window has been closed by the user
                _ => (),
            }
        }
        // window.swap_buffers();
        // std::thread::sleep(std::time::Duration::from_millis(17));
    }

}

#[macro_use]
extern crate glium;

extern crate slog_stdlog;
#[macro_use]
extern crate log;


fn main() {
    let args: Vec<String> = env::args().collect();
    let file = &args[1];
    let path = Path::new(file);
    let log = Logger::new_root(o!("version" => "0.1", "build-id" => "8dfljdf"));
    log.set_drain(slog_term::async_stdout());
    slog_stdlog::set_logger(log.clone()).unwrap();

    start(path)
}
