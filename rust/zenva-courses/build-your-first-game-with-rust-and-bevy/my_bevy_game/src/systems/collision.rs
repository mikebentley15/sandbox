use crate::components::*;
use bevy::math::bounding::{Aabb2d, BoundingCircle, BoundingVolume, IntersectsVolume};
use bevy::prelude::*;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Collision {
    Left,
    Right,
    Top,
    Bottom,
}

pub fn handle_collision(
    mut ball_query: Query<(&mut Velocity, &Position, &Shape), With<Ball>>,
    other_things: Query<(&Position, &Shape), Without<Ball>>,
) {
    if let Ok((mut ball_velocity, ball_position, ball_shape)) = ball_query.get_single_mut() {
        for (position, shape) in &other_things {
            if let Some(collision) = detect_collision(ball_position, ball_shape, position, shape) {
                respond_to_collision(&mut ball_velocity, collision);
            }
        }
    }
}

fn detect_collision(
    ball_position: &Position,
    ball_shape: &Shape,
    other_position: &Position,
    other_shape: &Shape,
) -> Option<Collision> {
    let ball = BoundingCircle::new(ball_position.0, ball_shape.0.x);
    let wall = Aabb2d::new(other_position.0, other_shape.0 / 2.0);

    if !ball.intersects(&wall) {
        return None;
    }

    let closest_point = wall.closest_point(ball.center());
    let offset = ball.center() - closest_point;
    if offset.x.abs() > offset.y.abs() {
        if offset.x < 0.0 {
            Some(Collision::Left)
        } else {
            Some(Collision::Right)
        }
    } else {
        if offset.y < 0.0 {
            Some(Collision::Top)
        } else {
            Some(Collision::Bottom)
        }
    }
}

fn respond_to_collision(velocity: &mut Velocity, collision: Collision) {
    match collision {
        Collision::Left | Collision::Right => velocity.0.x *= -1.0,
        Collision::Top | Collision::Bottom => velocity.0.y *= -1.0,
    }
}
