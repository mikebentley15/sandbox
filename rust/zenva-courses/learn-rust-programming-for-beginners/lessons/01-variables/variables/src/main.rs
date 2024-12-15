fn main() {
    variables();
    numbers();
    arrays();
    tuples();
    constants();
    strings();
    challenge();
}

fn variables() {
    // variable names are locations in memory
    let mut value = 9;
    println!("The value is {}", value);

    value = 10;
    println!("The value is {}", value);

    let x = 64;
    println!("The value of x is {}", x);

    let x = x + 1; // shadowing
    println!("The value of x is {}", x);

    let x = "RUST PROGRAMMING"; // shadowing can change a variable's type too
    println!("The value of x is {}", x);
}

fn numbers() {
    // you can specify a variables type
    let small_value: i8 = 100; // int types: i8, i16, i32, i64, u8, u16, u32, u64
    let float_value: f32 = -24.1; // float times: f32, f64
    println!("small value: {}", small_value);
    println!("float value: {}", float_value);
}

fn arrays() {
    // Arrays
    let numbers: [i32; 5] = [1, 2, 3, 4, 5];
    println!(
        "numbers: [{}, {}, {}, {}, {}]",
        numbers[0], numbers[1], numbers[2], numbers[3], numbers[4]
    );
}

fn tuples() {
    // Tuples
    let person = ("Alice", 30, 5.4);
    println!(
        "Name: {}, Age: {}, Salary: {}",
        person.0, person.1, person.2
    );
}

fn constants() {
    // Constants
    // - cannot use "mut"
    // - always must specify its type
    // - constants cannot be shadowed
    const PI: f32 = 3.14159;
    println!("PI: {}", PI);
}

fn strings() {
    // Two basic string types:
    // 1. &str: string slices
    //   - no ownership
    //   - immutable view into string data
    //   - default type of any string in quotes
    let greeting: &str = "Hello, world!";
    println!("{}", greeting);

    // 2. String: mutable dynamic strings
    //   - owning
    //   - heap allocated
    //   - can be mutable
    let mut name = String::from("Zenva");
    name.push_str(" Academy");
    println!("Name: {}", name);
}

fn challenge() {
    // Write a program that:
    //
    // 1. Stores a person’s name.
    // 2. Uses a variable to store the person’s current age.
    // 3. Calculates the person’s age in a future year using a constant for the number of years in the future.
    // 4. Prints the name and the calculated future age.
    //
    // At every step, think:
    //
    // - Whether it is best to use a mutable or immutable variable
    // - What would be the appropriate data type

    let name = "Henry";
    let current_age: u32 = 32;
    const FUTURE: u32 = 15;
    let future_age: u32 = current_age + FUTURE;
    println!(
        "{} will be {} years old {} years from now",
        name, future_age, FUTURE
    );
}
