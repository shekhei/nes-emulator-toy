use std::rc::Rc;
use std::cell::RefCell;

pub trait Read {
    fn load_b(&mut self, addr: u16) -> u8;
}

pub trait Write {
    fn save_b(&mut self, addr: u16, val: u8);
}

pub struct PPU<T: Read + Write, U: Write> {
    cycle: u8,
    // lets do a naive engine first
    playfield: Rc<RefCell<U>>, // actually just 2 bits per pixel
    regs: [u8; 8],
    vram: Vram<T>,
    scroll_x: u8,
    scroll_y: u8,
//     v
// Current VRAM address (15 bits)
// t
// Temporary VRAM address (15 bits); can also be thought of as the address of the top left onscreen tile.
// x
// Fine X scroll (3 bits)
// w
// First or second write toggle (1 bit)
    t: u16,
    v: u16,
    x: u8,
    w: bool,
    scanline: u8,
    frame: u16,
    oam: [u8; 0x100],
}

pub struct Vram<T: Read + Write> {
    mapped: Rc<RefCell<T>>,
    nametables: [u8; 0x1000],
    palette: [u8; 0x20],
}

impl<T: Read + Write> Vram<T> {
    pub fn new(mapped: Rc<RefCell<T>>) -> Vram<T> {
        Vram {
            mapped: mapped,
            nametables: [0;0x1000],
            palette: [0; 0x20],
        }
    }
}

impl<T: Read + Write> Read for Vram<T> {
    fn load_b(&mut self, addr: u16) -> u8 {
        if addr < 0x2000 {
            debug!("loading from rom");
            return self.mapped.borrow_mut().load_b(addr)
        } else if addr < 0x3eff {
            debug!("Loading from nametable");
            let mut offset = addr - 0x2000;
            offset %= 0x1000;
            return self.nametables[offset as usize]
        }
        debug!("loading from palette");
        let mut offset = addr - 0x3f00;
        offset %= 0x20;
        self.palette[offset as usize]
    }
}

impl<T: Read + Write> Write for Vram<T> {
    fn save_b(&mut self, addr: u16, val: u8) {
        if addr < 0x2000 {
            debug!("saving to rom {:x}", addr);
            self.mapped.borrow_mut().save_b(addr, val);
        } else if addr < 0x3eff {
            debug!("saving to nametable");
            let mut offset = addr - 0x2000;
            offset %= 0x1000;
            self.nametables[offset as usize] = val;
        } else {
            debug!("saving to palette");
            let mut offset = addr - 0x3f00;
            offset %= 0x20;
            self.palette[offset as usize] = val;
        }
    }
}

impl<T: Read + Write, U: Write> PPU<T, U> {
    pub fn new(vram: Vram<T>, playfield: Rc<RefCell<U>>) -> PPU<T, U> {
        PPU::<T, U>{
            cycle:0,
            playfield:playfield,
            regs: [
                0,
                0,
                0b10100000,
                0,
                0,
                0,
                0,
                0,
            ],
            vram: vram,
            scroll_x: 0,
            scroll_y: 0,
            t: 0,
            x: 0,
            v: 0,
            w: false,
            scanline: 0,
            frame: 0,
            oam: [0u8; 0x100],
        }

    }

    pub fn get_mut_oam_ref(&mut self) -> &mut [u8; 0x100] {
        return &mut self.oam;
    }
    fn incr_vram(&mut self) {
        // TODO: implement the weird while rendering increment behavior

        let ctrl = self.regs[0];
        if (ctrl & 0b100) == 0b100 {
            // increment by 32
            self.v.wrapping_add(32);
        } else {
            self.v.wrapping_add(1);
        }
    }
    pub fn load(&mut self, oper:u16) -> u8 {
        match oper & 7 {
            2=> self.read_ppustatus(),
            7 => {
                let v = self.v;
                let val = self.vram.load_b(v);
                self.incr_vram();
                return val
            },
            _ => self.regs[(oper & 7) as usize],
        }

    }
    pub fn save(&mut self, oper:u16, val: u8) {
        match oper & 7 {
            0 => self.update_ppuctrl(val),
            5 => self.update_ppuscroll(val),
            6 => self.update_addr(val),
            7 => {
                let v = self.v;
                self.vram.save_b(v, val);
                self.incr_vram();
            },
            _ => self.regs[(oper & 7) as usize] = val,
        };

    }
    pub fn update_ppuctrl(&mut self, val: u8) {
        let val = val as u16;
        self.t &= (val & 0b11) << 10;
    }
    pub fn update_ppuscroll(&mut self, val: u8) {
        // using w to note which part to write to
        let val = val as u16;
        if self.w == false {
            // t: ....... ...HGFED = d: HGFED...
            // x:              CBA = d: .....CBA
            // w:                  = 1
            self.t &= (val & 0b11111000) >> 3;
            self.x &= val as u8 & 0b00000111;
            self.w = true;
        } else {
            // t: CBA..HG FED..... = d: HGFEDCBA
            // w:                  = 0
            self.t &= (val & 0b11111000) << 2;
            self.t &= (val & 0b00000111) << 12;
            self.w = false;
        }
    }
    pub fn update_addr(&mut self, val: u8) {
        let val = val as u16;
        if self.w == false {
            // t: .FEDCBA ........ = d: ..FEDCBA
            // t: X...... ........ = 0
            // w:                  = 1
            self.t &= (val & 0b111111) << 8;
            self.t &= 0b011111111111111;
            self.w = true;
        } else {
            // t: ....... HGFEDCBA = d: HGFEDCBA
            // v                   = t
            // w:                  = 0
            self.t &= val;
            let t = self.t;
            self.v = t;
            self.w = false;
        }
    }
    pub fn read_ppustatus(&mut self) -> u8 {
        self.w = false;
        self.regs[2]
    }
    pub fn tick(&mut self) {
        self.cycle += 1;
        if self.cycle > 340 {
            self.cycle = 0;
            self.scanline += 1;
            if self.scanline > 261 {
                self.scanline = 0;
                self.frame.wrapping_add(1);
            }
        }
        let scanline = self.scanline;
        let preline = scanline == 261;
        let visible = scanline < 240;
        let render = preline || visible;
    }
    pub fn reset(&mut self) {
        self.regs[0] = 0;
        self.regs[1] = 0;
        self.regs[5] = 0;
        self.regs[7] = 0;
    }
}
