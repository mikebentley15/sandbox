use std::borrow::Borrow;
use std::cmp::Eq;
use std::collections::{HashMap, HashSet};
use std::hash::Hash;

pub struct Multimap<K, V> {
    pub data: HashMap<K, HashSet<V>>,
}

impl<K: Hash + Eq, V: Hash + Eq> Multimap<K, V> {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
        }
    }

    pub fn insert(&mut self, k: K, v: V) -> bool {
        match self.data.get_mut(&k) {
            Some(set) => set.insert(v),
            None => {
                let mut start_set = HashSet::new();
                start_set.insert(v);
                self.data.insert(k, start_set);
                true
            }
        }
    }

    pub fn contains_key_and_value<QK, QV>(&self, k: &QK, v: &QV) -> bool
    where
        K: Borrow<QK>,
        QK: Hash + Eq + ?Sized,
        V: Borrow<QV>,
        QV: Hash + Eq + ?Sized,
    {
        self.data
            .get(k)
            .map(|vals| vals.contains(v))
            .unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multimap_creation_is_empty() {
        let map: Multimap<u32, u32> = Multimap::new();
        assert_eq!(0, map.data.len());
    }

    #[test]
    fn multimap_insertion_of_new_key() {
        let mut map: Multimap<u32, u32> = Multimap::new();
        assert!(map.insert(5, 10));
        assert_eq!(1, map.data.len());
        assert_eq!(
            HashSet::from([5u32]),
            map.data.keys().cloned().collect::<HashSet<_>>()
        );
        let value = map.data.get(&5).expect("5 should be in the set");
        assert_eq!(HashSet::from([10u32]), value.clone());
    }

    #[test]
    fn multimap_insertion_of_existing_key_but_new_value() {
        let mut map: Multimap<u32, u32> = Multimap::new();
        assert!(map.insert(5, 10));
        assert!(map.insert(5, 15));
        assert_eq!(1, map.data.len());
        assert_eq!(
            HashSet::from([5u32]),
            map.data.keys().cloned().collect::<HashSet<_>>()
        );
        let value = map.data.get(&5).expect("5 should be in the set");
        assert_eq!(HashSet::from([10u32, 15u32]), value.clone());
    }

    #[test]
    fn multimap_insertion_of_existing_key_and_value() {
        let mut map: Multimap<u32, u32> = Multimap::new();
        assert!(map.insert(5, 10));
        assert!(!map.insert(5, 10));
        assert_eq!(1, map.data.len());
        assert_eq!(
            HashSet::from([5u32]),
            map.data.keys().cloned().collect::<HashSet<_>>()
        );
        let value = map.data.get(&5).expect("5 should be in the set");
        assert_eq!(HashSet::from([10u32]), value.clone());
    }
}
