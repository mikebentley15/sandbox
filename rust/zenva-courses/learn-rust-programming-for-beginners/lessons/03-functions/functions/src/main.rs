fn main() {
    greet_user(String::from("Alice"));
    println!("Sum: {}", add(4, 5));
    challenge_student_grades();
}

fn greet_user(name: String) {
    println!("Hello {}, welcome to Rust programming.", name);
}

fn add(x: i32, y: i32) -> i32 {
    let sum = x + y; // lines with semicolons are called statements
    sum // lines without semicolons are called returns
}

fn challenge_student_grades() {
    // 1. Create a function named get_letter_grade that takes an integer representing a student’s
    //    score and returns the corresponding letter grade as a character. The grading scale is as
    //    follows:
    //   - A: 90-100
    //   - B: 80-89
    //   - C: 70-79
    //   - D: 60-69
    //   - F: Below 60
    // 2. In the main function, create an array of student scores(e.g., [85, 90, 78, 92, 88]).
    // 3. Use a for loop to iterate through the array and use the get_letter_grade function to
    //    determine the letter grade for each score.
    // 4. Print each student’s score along with the corresponding letter grade.
    let scores: [u8; 5] = [85, 90, 78, 92, 88];
    for score in scores {
        println!("{}: {}", score, get_letter_grade(score));
    }
}

fn get_letter_grade(score: u8) -> char {
    if score < 60 {
        'F'
    } else if score < 70 {
        'D'
    } else if score < 80 {
        'C'
    } else if score < 90 {
        'B'
    } else {
        'A'
    }
}
