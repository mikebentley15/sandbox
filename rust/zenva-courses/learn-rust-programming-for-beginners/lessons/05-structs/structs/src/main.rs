fn main() {
    tuple_structs();
    structs_with_methods();
    struct_challenge();
    regular_enums();
    enums_with_values();
    challenge_parking_prices();
}

// Tuple structs don't have field names
#[derive(Debug)] // used to print out with debug macros
struct Color(u8, u8, u8); // red, green, blue
#[derive(Debug)] // used to print out with debug macros
struct Point2(f64, f64); // x, y

fn tuple_structs() {
    let red = Color(255, 0, 0);
    let coord = Point2(25.5, 13.2);
    println!("{red:?}, {coord:?}");
    println!("red color:  {}, {}, {}", red.0, red.1, red.2); // access like a tuple
    println!("coordinate: {}, {}", coord.0, coord.1);
}

struct Rectangle {
    width: f32,
    height: f32,
}

// Methods
impl Rectangle {
    fn area(&self) -> f32 {
        return self.width * self.height;
    }

    fn print_dimensions(&self) {
        println!("Rectangle(width: {}, height: {})", self.width, self.height);
    }
}

fn structs_with_methods() {
    let rect = Rectangle {
        width: 32.0,
        height: 16.0,
    };
    rect.print_dimensions();
    println!("Rectangle area: {}", rect.area());
}

struct Book {
    title: String,
    author: String,
    page_count: u32,
}

impl Book {
    fn get_summary(&self) -> String {
        format!(
            "Book(title: {}, author: {}, # pages: {})",
            self.title, self.author, self.page_count
        )
    }
}

fn struct_challenge() {
    // 1. Define a Book struct with fields for the title, author, and number of pages.
    // 2. Implement a method on the Book struct:
    //     - get_summary to return a summary of the book as a string.
    // 3. In the main function, create an array of books.
    // 4. Use a for loop to iterate through the array and print the summary of each book using the get_summary method.
    let books = [
        Book {
            title: String::from("Go Sam Go"),
            author: String::from("unknown"),
            page_count: 15,
        },
        Book {
            title: String::from("Ivy From Mom Dad"),
            author: String::from("Ivy"),
            page_count: 6,
        },
        Book {
            title: String::from("Big Buns"),
            author: String::from("Obsessed with Buns"),
            page_count: 15432,
        },
    ];

    for book in books {
        println!("{}", book.get_summary());
    }
}

// Regular enum that can have one of many distinct values
enum TrafficLight {
    Red,
    Yellow,
    Green,
}

fn regular_enums() {
    let light = TrafficLight::Red;

    /*Pattern Matching -> match */
    match light {
        TrafficLight::Red => println!("Stop!"),
        TrafficLight::Yellow => println!("Caution! Be prepared to stop"),
        TrafficLight::Green => println!("Go!!"),
    }
}

// Enums with values: basically a variant of structs
// Structs can have named fields or can be tuple structs.
// Option<T> and Result<T, E> are enums with values
enum Shape {
    Circle(f64),
    Rectangle { w: f64, h: f64 },
    Square(i32),
}

fn calculate_area(shape: Shape) {
    // Extract the variant values within the match clause.
    match shape {
        Shape::Circle(radius) => println!("Area of circle is {}", 3.14 * radius * radius),
        Shape::Rectangle { w, h } => {
            let area = w * h;
            println!("Area of rectangle is {}", area);
        }
        Shape::Square(side) => println!("Area of square is {}", side * side),
    }
}

fn enums_with_values() {
    let rectangle1 = Shape::Rectangle { w: 30.0, h: 17.3 };
    let square = Shape::Square(25);
    calculate_area(rectangle1);
    calculate_area(square);
}

#[derive(Debug)] // used to print out with debug macros
enum CarType {
    SUV,
    Sedan,
    Coupe,
}

#[derive(Debug)] // used to print out with debug macros
enum Vehicle {
    Car(CarType),
    Truck(u8), // tons
    Motorcycle,
}

impl Vehicle {
    fn parking_rate(&self) -> u32 {
        match self {
            Vehicle::Car(CarType::SUV) => 20,
            Vehicle::Car(CarType::Sedan) => 15,
            Vehicle::Car(CarType::Coupe) => 10,
            Vehicle::Truck(tons) => {
                if *tons > 10 {
                    25
                } else {
                    20
                }
            }
            Vehicle::Motorcycle => 10,
        }
    }
}

fn challenge_parking_prices() {
    // You are developing a program to charge parking price for different types of vehicles in a
    // parking lot. Each vehicle can be a Car, Truck, or Motorcycle. Cars can be of different types
    // such as an SUV, Sedan, or Coupe. Trucks have the cargo capacity in tons.
    //
    // Your task is to create enums to represent the different types of vehicles and car types, use
    // pattern matching to calculate parking rates, and then use these in the main program to print
    // the parking rates for any vehicle(s) of your choice.
    //
    // | Vehicle Type | Sub Type  | Parking Rate ($) |
    // |--------------|-----------|------------------|
    // | Car          | SUV       | 20               |
    // | Car          | Sedan     | 15               |
    // | Car          | Coupe     | 10               |
    // | Truck        | > 10 tons | 25               |
    // | Truck        | ≤ 10 tons | 20               |
    // | Motorcycle   |           | 10               |
    //
    // Task:
    //
    // 1. Define an enum named CarType with variants SUV, Sedan, and Coupe.
    // 2. Define an enum named Vehicle with variants Car, Truck, and Motorcycle.
    // 3. Implement a method on the Vehicle enum named parking_rate that calculates and returns the
    //    parking rate for each vehicle.
    // 4. In the main function, create a few vehicles.
    // 5. Print the parking rate of the vehicle using the parking_rate method.
    let vehicles = [
        Vehicle::Car(CarType::Coupe),
        Vehicle::Car(CarType::Sedan),
        Vehicle::Car(CarType::SUV),
        Vehicle::Truck(5),
        Vehicle::Truck(10),
        Vehicle::Truck(15),
        Vehicle::Motorcycle,
    ];
    for vehicle in vehicles {
        println!(
            "for {vehicle:?}, the parking rate is {}",
            vehicle.parking_rate()
        );
    }
}
