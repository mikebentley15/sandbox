use regex::Regex;
use soln::common::*;

fn main() {
    let fname = get_input_file();
    let mul_lines = read_lines(&fname).unwrap();
    let total: usize = mul_lines.map(|line| sum_of_muls(line.unwrap())).sum();
    println!("\nTotals to {}", total);
}

fn sum_of_muls<S: AsRef<str>>(mul_string: S) -> usize {
    let re = Regex::new(r"mul\(([1-9][0-9]{0,2}),([1-9][0-9]{0,2})\)").unwrap();
    return re
        .captures_iter(mul_string.as_ref())
        .map(|cap| parse_mul(&cap[1], &cap[2]))
        .sum();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sum_of_muls() {
        assert_eq!(sum_of_muls(""), 0);

        assert_eq!(sum_of_muls("mul(1,1)"), 1);
        assert_eq!(sum_of_muls("mul(2,4)"), 2 * 4);
        assert_eq!(sum_of_muls("mul(44,46)"), 44 * 46);
        assert_eq!(sum_of_muls("mul(123,4)"), 123 * 4);
        assert_eq!(sum_of_muls("mul(1,1)mul(3,3)"), 1 + 3 * 3);

        assert_eq!(sum_of_muls("no muls here"), 0); // no muls
        assert_eq!(sum_of_muls("mul(-1,1)"), 0); // negatives not allowed
        assert_eq!(sum_of_muls("mul(12345,123)"), 0); // number too big
        assert_eq!(sum_of_muls("mul(01,1)"), 0); // numbers don't start with zero

        assert_eq!(sum_of_muls("mul(12mul(345,123)"), 345 * 123);
        assert_eq!(
            sum_of_muls("xmul(2,4)%&mul[3,7]!@^do_not_mul(5,5)+mul(32,64]then(mul(11,8)mul(8,5))"),
            161
        );
    }
}
