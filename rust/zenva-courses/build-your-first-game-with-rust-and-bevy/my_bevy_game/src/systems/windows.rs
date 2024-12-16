use crate::constants;
use bevy::prelude::*;

pub fn create_window() -> WindowPlugin {
    WindowPlugin {
        primary_window: Some(Window {
            title: "My Pong Game".to_string(),
            ..default() // defaults for the remaining fields
        }),
        ..default() // defaults for the remaining fields
    }
}

pub fn spawn_dotted_line(mut commands: Commands) {
    let dot_color = Color::srgb(0.5, 0.5, 0.5);
    let dot_size = Vec3::new(5.0, 20.0, 1.0);
    let gap_size = 20.0;
    let num_dots = (constants::WINDOW_HEIGHT / (dot_size.y + gap_size)) as i32;

    for i in 0..num_dots {
        let ypos = i as f32 * (dot_size.y + gap_size) - constants::WINDOW_HEIGHT / 2.0;
        commands.spawn((
            Sprite {
                color: dot_color,
                ..default()
            },
            Transform {
                translation: Vec3::new(0.0, ypos, 0.0),
                scale: dot_size,
                ..default()
            },
        ));
    }
}
