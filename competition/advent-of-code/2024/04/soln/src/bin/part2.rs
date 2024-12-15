use regex::Regex;
use soln::common::*;

fn main() {
    let fname = get_input_file();
    let mul_string = std::fs::read_to_string(&fname).unwrap();
    let total = sum_of_muls_with_conditionals(mul_string);
    println!("\nTotals to {}", total);
}

pub fn sum_of_muls_with_conditionals<S: AsRef<str>>(mul_string: S) -> usize {
    let command_re =
        Regex::new(r"do\(\)|don't\(\)|mul\(([1-9][0-9]{0,2}),([1-9][0-9]{0,2})\)").unwrap();
    let mut enabled = true;
    let mut total: usize = 0;
    for cap in command_re.captures_iter(mul_string.as_ref()) {
        match &cap[0] {
            "do()" => enabled = true,
            "don't()" => enabled = false,
            _ => {
                if enabled {
                    total += parse_mul(&cap[1], &cap[2]);
                } else {
                    println!("Disabled {} * {}", &cap[1], &cap[2]);
                }
            }
        }
    }
    return total;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_regex_expectations() {
        let re =
            Regex::new(r"do\(\)|don't\(\)|mul\(([1-9][0-9]{0,2}),([1-9][0-9]{0,2})\)").unwrap();
        assert_eq!(re.captures("do()").unwrap()[0], *"do()");
        assert_eq!(re.captures("don't()").unwrap()[0], *"don't()");
        assert_eq!(re.captures("mul(1,1)").unwrap()[0], *"mul(1,1)");
    }

    #[test]
    fn test_sum_of_muls_with_conditionals() {
        assert_eq!(sum_of_muls_with_conditionals(""), 0);

        assert_eq!(sum_of_muls_with_conditionals("mul(1,1)"), 1);
        assert_eq!(sum_of_muls_with_conditionals("mul(2,4)"), 2 * 4);
        assert_eq!(sum_of_muls_with_conditionals("mul(44,46)"), 44 * 46);
        assert_eq!(sum_of_muls_with_conditionals("mul(123,4)"), 123 * 4);
        assert_eq!(sum_of_muls_with_conditionals("mul(1,1)mul(3,3)"), 1 + 3 * 3);

        assert_eq!(sum_of_muls_with_conditionals("no muls here"), 0); // no muls
        assert_eq!(sum_of_muls_with_conditionals("mul(-1,1)"), 0); // negatives not allowed
        assert_eq!(sum_of_muls_with_conditionals("mul(12345,123)"), 0); // number too big
        assert_eq!(sum_of_muls_with_conditionals("mul(01,1)"), 0); // numbers don't start with zero

        assert_eq!(
            sum_of_muls_with_conditionals("mul(12mul(345,123)"),
            345 * 123
        );
        assert_eq!(
            sum_of_muls_with_conditionals(
                "xmul(2,4)%&mul[3,7]!@^do_not_mul(5,5)+mul(32,64]then(mul(11,8)mul(8,5))"
            ),
            161
        );

        assert_eq!(sum_of_muls_with_conditionals(r"xdon't()mul(1,1)"), 0);
        assert_eq!(
            sum_of_muls_with_conditionals("xdon't()mul(1,1)do()mul(2,2)"),
            4
        );
        assert_eq!(
            sum_of_muls_with_conditionals(
                r"xmul(2,4)&mul[3,7]!^don't()\_mul(5,5)+mul(32,64](mul(11,8)undo()?mul(8,5))"
            ),
            48
        );
    }
}
