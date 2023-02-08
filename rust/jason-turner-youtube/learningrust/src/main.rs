#[derive(Debug)]
struct Position {
    x: i8,
    y: i8,
}

impl Position {
    fn update_y(&mut self) {
        self.y -= 10;
    }

    fn update_x(&mut self) {
        self.x += 1;
        self.update_y();
    }
}

fn main1() {
    // const is by default
    let mut pos = Position { x: 42, y: 32 };
    // pos.x = 12; // would fail if pos is created w/o mut keyword
    pos.y = 15;
    let pos2 = &mut pos;
    pos2.x = 12;
    //pos.y = 18; // not allowed since pos has been "borrowed"
    //let pos3 = &pos; // cannot do an immutable borrow of a mutable type
    //let pos3 = &mut pos; // cannot borrow mutably more than once
    //pos3.x = 16;
    pos2.update_x();
    Position::update_x(pos2); // alternative way to call
    //println!("pos : {:#?}", pos); // not allowed to use since it's been borrowed
    println!("pos2: {:#?}", pos2);
}

fn main() {
    
}