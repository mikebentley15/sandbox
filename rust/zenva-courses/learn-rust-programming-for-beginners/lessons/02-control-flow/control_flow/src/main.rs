fn main() {
    divisibility_check(35);
    divisibility_check(30);
    divisibility_check(24);
    divisibility_check(240);
    print_activity(true);
    print_activity(false);
    for_loop();
    while_loop();
    loop_loop();
    fibonacci_challenge(20);
    average_temperature_challenge([70.1, 75.2, 72.3, 68.4, 74.5, 78.6, 73.7]);
}

fn divisibility_check(number: i32) {
    // If a number is divisible by 3 and 5, print "TriQunit"
    // If a number is divisible by 4 and 6, print "HexaQuad"
    // Otherwise, print "just another number"
    if number % 5 == 0 && number % 3 == 0 {
        println!("{} is a TriQunit", number);
    } else if number % 6 == 0 && number % 4 == 0 {
        println!("{} is a HexaQuad", number);
    } else {
        println!("{} is just another number", number);
    }
}

fn print_activity(is_weekend: bool) {
    let activity = if is_weekend {
        "Go hiking!"
    } else {
        "Go to work!"
    };
    println!("Activity: {} (is weekend: {})", activity, is_weekend);
}

fn for_loop() {
    let arr = [10, 20, 30, 40, 50];
    for x in arr {
        println!("{}", x);
    }
}

fn while_loop() {
    let mut counter = 10;
    while counter > 0 {
        println!("Countdown: {}", counter);
        counter -= 1;
    }
    println!("LIFT OFF!");
}

fn loop_loop() {
    // loop is an infinite loop that can exit only with break
    let mut index = 1;
    loop {
        index += 1;
        println!("Index: {}", index);

        const MAX_INDEX: i32 = 15;
        if index == MAX_INDEX {
            println!("Max index reached");
            break;
        }
    }
}

fn fibonacci_challenge(count: u32) {
    // You are building a program to generate and print the first 10 numbers in the Fibonacci
    // sequence. The Fibonacci sequence is a series of numbers where each number is the sum of the
    // two preceding ones, usually starting with 0 and 1. The sequence looks like this
    //
    //   0, 1, 1, 2, 3, 5, 8, 13, 21, 34, ...
    //
    // Write a Rust program that uses a loop to generate and print the first 10 numbers in the
    // Fibonacci sequence.

    const FIRST: u32 = 0;
    const SECOND: u32 = 1;
    let mut a: u32 = FIRST;
    let mut b: u32 = SECOND;

    print!("First {} numbers of the Fibonacci sequence: {}", count, a);
    for _ in 1..count {
        print!(", {}", b);
        (a, b) = (b, a + b);
    }
    println!("");
}

fn average_temperature_challenge(arr: [f32; 7]) {
    // 1. Create an array with the temperatures for each day of the week (e.g., [70, 75, 72, 68, 74, 78, 73]).
    // 2. Use a for loop to iterate through the array and calculate the total sum of the temperatures.
    // 3. Calculate the average temperature.
    // 4. Print the total sum and the average temperature.
    let mut sum: f32 = 0.0;
    for val in arr {
        sum += val;
    }
    let avg = sum / (arr.len() as f32);
    println!("Total temperature: {}, Average temperature: {}", sum, avg);
}
