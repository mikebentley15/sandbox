use bevy::math::Vec2;
use bevy::prelude::*;

// No fields, just used as a marker components

#[derive(Component)]
pub struct Player1;

#[derive(Component)]
pub struct Player2;

#[derive(Component)]
pub struct Player1Score;

#[derive(Component)]
pub struct Player2Score;

pub enum Scorer {
    Player1,
    Player2,
}

#[derive(Component)]
pub struct Ball;

#[derive(Component)]
pub struct Paddle;

#[derive(Component)]
pub struct Boundary;

#[derive(Component)]
pub struct Position(pub Vec2);

#[derive(Component)]
pub struct Velocity(pub Vec2);

#[derive(Component)]
pub struct Shape(pub Vec2);

#[derive(Event)]
pub struct Scored(pub Scorer);

#[derive(Resource, Default)]
pub struct Score {
    pub player1: u32,
    pub player2: u32,
}
