fn main() {
    ownership_demonstration();
    slices();
}

fn ownership_demonstration() {
    let s1 = String::from("hello"); // s1 owns hello
    let s2 = s1; // ownership has transferred to s2

    // println!("{s1}"); // error! s1 doesn't own any data
    println!("{s2}"); // okay, s2 owns the data

    let s3 = s2.clone(); // s3 is a copy of s2.  s2 still owns the original hello.
    println!("{s2}"); // okay, s2 owns hello
    println!("{s3}"); // okay, s3 owns a copy of hello

    let name1 = String::from("Henry");
    print_string_1(name1); // ownership transferred to print_string

    // println!("{name}"); // error! name1 doesn't own Henry

    let name2 = String::from("Bob");
    let name2 = print_string_2(name2); // ownership transferred, and returned.  Use shadowing
    println!("{name2}"); // okay: name2's shadow declaration owns the data again

    let name3 = String::from("Mike");
    print_string_from_ref(&name3);
    println!("{name3}");

    // Ownership rules do not happen for basic data types like integers
    let j = 9;
    let k = j;
    println!("{j}, {k}"); // okay, no need to manage ownership of integers
}

fn print_string_1(string: String) {
    println!("{string}");
}

fn print_string_2(string: String) -> String {
    println!("{string}");
    string // Return the ownership back to the caller
}

fn print_string_from_ref(string: &String) {
    println!("{string}");
}

fn slices() {
    println!("\nslices()");
    let message = String::from("hello world");
    // Slices allow you to access a subset of a sequence
    let hello = &message[0..5]; // characters from the beginning until (and not including) index 5.
    println!("{message}");
    println!("{hello}");

    let arr = [1, 2, 3, 4, 5];
    let slice = &arr[1..4]; // [2, 3, 4]
    for x in slice {
        print!("{x}, ");
    }
    println!("");
    println!("{:?}", slice);
}
