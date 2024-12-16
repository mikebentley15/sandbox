use crate::components::*;
use bevy::prelude::*;

pub fn spawn_scoreboard(mut commands: Commands) {
    // Broken in upgrade from bevy 0.14 to 0.15
    // commands.spawn((
    //     TextBundle::from_section(
    //         "0",
    //         TextStyle {
    //             font_size: 64.0,
    //             color: Color::WHITE,
    //             ..default()
    //         },
    //     )
    //     .with_text_justify(JustifyText::Center)
    //     .with_style(Style {
    //         position_type: PositionType::Absolute,
    //         top: Val::Px(5.0),
    //         left: Val::Percent(45.0),
    //         ..default()
    //     }),
    //     Player1Score,
    // ));
    // commands.spawn((
    //     TextBundle::from_section(
    //         "0",
    //         TextStyle {
    //             font_size: 64.0,
    //             color: Color::WHITE,
    //             ..default()
    //         },
    //     )
    //     .with_text_justify(JustifyText::Center)
    //     .with_style(Style {
    //         position_type: PositionType::Absolute,
    //         top: Val::Px(5.0),
    //         right: Val::Percent(45.0),
    //         ..default()
    //     }),
    //     Player2Score,
    // ));
}

// pub fn update_scoreboard(
//     mut player1_score: Query<&mut Text, With<Player1Score>>,
//     mut player2_score: Query<&mut Text, With<Player2Score>>,
//     score: Res<Score>,
// ) {
//     // Broken in upgrade from bevy 0.14 to 0.15
//     // if score.is_changed() {
//     //     if let Ok(mut player1_score) = player1_score.get_single_mut() {
//     //         player1_score.section[0].value = score.player1.to_string();
//     //     }
//     //     if let Ok(mut player2_score) = player2_score.get_single_mut() {
//     //         player2_score.section[0].value = score.player2.to_string();
//     //     }
//     // }
// }

pub fn update_score(mut score: Res<Score>, mut events: EventReader<Scored>) {
    // Broken in upgrade from bevy 0.14 to 0.15
    // for event in events.read() {
    //     match event.0 {
    //         Scorer::Player1 => score.player1 += 1,
    //         Scorer::Player2 => score.player2 += 1,
    //     }
    // }
}
