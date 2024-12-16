use bevy::prelude::*;
mod systems;
use components::*;
use systems::*;
mod bundles;
mod components;
mod constants;
use bundles::*;

fn main() {
    // By following instructions here:
    //   https://bevyengine.org/learn/quick-start/getting-started/setup/
    // Incremental build
    // - before: 1m20s
    // - after `cargo add bevy -F dynamic_linking`: 1.5s
    // - after configuring lld for linking: 1.5s
    println!("Pong app starting...");

    App::new()
        .add_plugins(DefaultPlugins.set(create_window()))
        .init_resource::<Score>()
        .add_event::<Scored>()
        .add_systems(
            Startup,
            (
                spawn_dotted_line,
                spawn_camera,
                spawn_ball,
                spawn_paddles,
                spawn_scoreboard,
            ),
        )
        .add_systems(
            Update,
            (
                move_ball,
                update_entity_positions.after(move_ball),
                detect_scoring,
                update_score.after(detect_scoring),
                // update_scoreboard.after(detect_scoring),
                move_player1_paddle,
                move_player2_paddle,
                move_paddles.after(move_player1_paddle),
                handle_collision.after(move_ball),
            ),
        )
        .run();
}
