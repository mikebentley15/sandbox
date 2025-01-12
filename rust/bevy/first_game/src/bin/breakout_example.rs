//! A simplified implementation of the classic game "Breakout".
//!
//! Demonstrates Bevy's stepping capabilities if compiled with the `bevy_debug_stepping` feature.

use bevy::{
    math::bounding::{Aabb2d, BoundingCircle, BoundingVolume, IntersectsVolume},
    prelude::*,
};

mod stepping {
    use bevy::{app::MainScheduleOrder, ecs::schedule::*, prelude::*};

    /// Independent [`Schedule`] for stepping systems.
    ///
    /// The stepping systems must run in their own schedule to be able to inspect
    /// all the other schedules in the [`App`].  This is because the currently
    /// executing schedule is removed from the [`Schedules`] resource while it is
    /// being run.
    #[derive(Debug, Hash, PartialEq, Eq, Clone, ScheduleLabel)]
    struct DebugSchedule;

    /// Plugin to add a stepping UI to an example
    #[derive(Default)]
    pub struct SteppingPlugin {
        schedule_labels: Vec<InternedScheduleLabel>,
        top: Val,
        left: Val,
    }

    impl SteppingPlugin {
        /// add a schedule to be stepped when stepping is enabled
        pub fn add_schedule(mut self, label: impl ScheduleLabel) -> SteppingPlugin {
            self.schedule_labels.push(label.intern());
            self
        }

        /// Set the location of the stepping UI when activated
        pub fn at(self, left: Val, top: Val) -> SteppingPlugin {
            SteppingPlugin { top, left, ..self }
        }
    }

    impl Plugin for SteppingPlugin {
        fn build(&self, app: &mut App) {
            app.add_systems(Startup, build_stepping_hint);
            if cfg!(not(feature = "bevy_debug_stepping")) {
                return;
            }

            // create and insert our debug schedule into the main schedule order.
            // We need an independent schedule so we have access to all other
            // schedules through the `Stepping` resource
            app.init_schedule(DebugSchedule);
            let mut order = app.world_mut().resource_mut::<MainScheduleOrder>();
            order.insert_after(Update, DebugSchedule);

            // create our stepping resource
            let mut stepping = Stepping::new();
            for label in &self.schedule_labels {
                stepping.add_schedule(*label);
            }
            app.insert_resource(stepping);

            // add our startup & stepping systems
            app.insert_resource(State {
                ui_top: self.top,
                ui_left: self.left,
                systems: Vec::new(),
            })
            .add_systems(
                DebugSchedule,
                (
                    build_ui.run_if(not(initialized)),
                    handle_input,
                    update_ui.run_if(initialized),
                )
                    .chain(),
            );
        }
    }

    /// Struct for maintaining stepping state
    #[derive(Resource, Debug)]
    struct State {
        // vector of schedule/nodeid -> text index offset
        systems: Vec<(InternedScheduleLabel, NodeId, usize)>,

        // ui positioning
        ui_top: Val,
        ui_left: Val,
    }

    /// condition to check if the stepping UI has been constructed
    fn initialized(state: Res<State>) -> bool {
        !state.systems.is_empty()
    }

    const FONT_COLOR: Color = Color::srgb(0.2, 0.2, 0.2);
    const FONT_BOLD: &str = "fonts/FiraSans-Bold.ttf";

    #[derive(Component)]
    struct SteppingUi;

    /// Construct the stepping UI elements from the [`Schedules`] resource.
    ///
    /// This system may run multiple times before constructing the UI as all of the
    /// data may not be available on the first run of the system.  This happens if
    /// one of the stepping schedules has not yet been run.
    fn build_ui(
        mut commands: Commands,
        asset_server: Res<AssetServer>,
        schedules: Res<Schedules>,
        mut stepping: ResMut<Stepping>,
        mut state: ResMut<State>,
    ) {
        let mut text_spans = Vec::new();
        let mut always_run = Vec::new();

        let Ok(schedule_order) = stepping.schedules() else {
            return;
        };

        // go through the stepping schedules and construct a list of systems for
        // each label
        for label in schedule_order {
            let schedule = schedules.get(*label).unwrap();
            text_spans.push((
                TextSpan(format!("{label:?}\n")),
                TextFont {
                    font: asset_server.load(FONT_BOLD),
                    ..default()
                },
                TextColor(FONT_COLOR),
            ));

            // grab the list of systems in the schedule, in the order the
            // single-threaded executor would run them.
            let Ok(systems) = schedule.systems() else {
                return;
            };

            for (node_id, system) in systems {
                // skip bevy default systems; we don't want to step those
                if system.name().starts_with("bevy") {
                    always_run.push((*label, node_id));
                    continue;
                }

                // Add an entry to our systems list so we can find where to draw
                // the cursor when the stepping cursor is at this system
                // we add plus 1 to account for the empty root span
                state.systems.push((*label, node_id, text_spans.len() + 1));

                // Add a text section for displaying the cursor for this system
                text_spans.push((
                    TextSpan::new("   "),
                    TextFont::default(),
                    TextColor(FONT_COLOR),
                ));

                // add the name of the system to the ui
                text_spans.push((
                    TextSpan(format!("{}\n", system.name())),
                    TextFont::default(),
                    TextColor(FONT_COLOR),
                ));
            }
        }

        for (label, node) in always_run.drain(..) {
            stepping.always_run_node(label, node);
        }

        commands
            .spawn((
                Text::default(),
                SteppingUi,
                Node {
                    position_type: PositionType::Absolute,
                    top: state.ui_top,
                    left: state.ui_left,
                    padding: UiRect::all(Val::Px(10.0)),
                    ..default()
                },
                BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.33)),
                Visibility::Hidden,
            ))
            .with_children(|p| {
                for span in text_spans {
                    p.spawn(span);
                }
            });
    }

    fn build_stepping_hint(mut commands: Commands) {
        let hint_text = if cfg!(feature = "bevy_debug_stepping") {
            "Press ` to toggle stepping mode (S: step system, Space: step frame)"
        } else {
            "Bevy was compiled without stepping support. Run with `--features=bevy_debug_stepping` to enable stepping."
        };
        info!("{}", hint_text);
        // stepping description box
        commands.spawn((
            Text::new(hint_text),
            TextFont {
                font_size: 15.0,
                ..default()
            },
            TextColor(FONT_COLOR),
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(5.0),
                left: Val::Px(5.0),
                ..default()
            },
        ));
    }

    fn handle_input(keyboard_input: Res<ButtonInput<KeyCode>>, mut stepping: ResMut<Stepping>) {
        if keyboard_input.just_pressed(KeyCode::Slash) {
            info!("{:#?}", stepping);
        }
        // grave key to toggle stepping mode for the FixedUpdate schedule
        if keyboard_input.just_pressed(KeyCode::Backquote) {
            if stepping.is_enabled() {
                stepping.disable();
                debug!("disabled stepping");
            } else {
                stepping.enable();
                debug!("enabled stepping");
            }
        }

        if !stepping.is_enabled() {
            return;
        }

        // space key will step the remainder of this frame
        if keyboard_input.just_pressed(KeyCode::Space) {
            debug!("continue");
            stepping.continue_frame();
        } else if keyboard_input.just_pressed(KeyCode::KeyS) {
            debug!("stepping frame");
            stepping.step_frame();
        }
    }

    fn update_ui(
        mut commands: Commands,
        state: Res<State>,
        stepping: Res<Stepping>,
        ui: Single<(Entity, &Visibility), With<SteppingUi>>,
        mut writer: TextUiWriter,
    ) {
        // ensure the UI is only visible when stepping is enabled
        let (ui, vis) = *ui;
        match (vis, stepping.is_enabled()) {
            (Visibility::Hidden, true) => {
                commands.entity(ui).insert(Visibility::Inherited);
            }
            (Visibility::Hidden, false) | (_, true) => (),
            (_, false) => {
                commands.entity(ui).insert(Visibility::Hidden);
            }
        }

        // if we're not stepping, there's nothing more to be done here.
        if !stepping.is_enabled() {
            return;
        }

        let (cursor_schedule, cursor_system) = match stepping.cursor() {
            // no cursor means stepping isn't enabled, so we're done here
            None => return,
            Some(c) => c,
        };

        for (schedule, system, text_index) in &state.systems {
            let mark = if &cursor_schedule == schedule && *system == cursor_system {
                "-> "
            } else {
                "   "
            };
            *writer.text(ui, *text_index) = mark.to_string();
        }
    }
}

// These constants are defined in `Transform` units.
// Using the default 2D camera they correspond 1:1 with screen pixels.
const PADDLE_SIZE: Vec2 = Vec2::new(120.0, 20.0);
const GAP_BETWEEN_PADDLE_AND_FLOOR: f32 = 60.0;
const PADDLE_SPEED: f32 = 500.0;
// How close can the paddle get to the wall
const PADDLE_PADDING: f32 = 10.0;

// We set the z-value of the ball to 1 so it renders on top in the case of overlapping sprites.
const BALL_STARTING_POSITION: Vec3 = Vec3::new(0.0, -50.0, 1.0);
const BALL_DIAMETER: f32 = 30.;
const BALL_SPEED: f32 = 400.0;
const INITIAL_BALL_DIRECTION: Vec2 = Vec2::new(0.5, -0.5);

const WALL_THICKNESS: f32 = 10.0;
// x coordinates
const LEFT_WALL: f32 = -450.;
const RIGHT_WALL: f32 = 450.;
// y coordinates
const BOTTOM_WALL: f32 = -300.;
const TOP_WALL: f32 = 300.;

const BRICK_SIZE: Vec2 = Vec2::new(100., 30.);
// These values are exact
const GAP_BETWEEN_PADDLE_AND_BRICKS: f32 = 270.0;
const GAP_BETWEEN_BRICKS: f32 = 5.0;
// These values are lower bounds, as the number of bricks is computed
const GAP_BETWEEN_BRICKS_AND_CEILING: f32 = 20.0;
const GAP_BETWEEN_BRICKS_AND_SIDES: f32 = 20.0;

const SCOREBOARD_FONT_SIZE: f32 = 33.0;
const SCOREBOARD_TEXT_PADDING: Val = Val::Px(5.0);

const BACKGROUND_COLOR: Color = Color::srgb(0.9, 0.9, 0.9);
const PADDLE_COLOR: Color = Color::srgb(0.3, 0.3, 0.7);
const BALL_COLOR: Color = Color::srgb(1.0, 0.5, 0.5);
const BRICK_COLOR: Color = Color::srgb(0.5, 0.5, 1.0);
const WALL_COLOR: Color = Color::srgb(0.8, 0.8, 0.8);
const TEXT_COLOR: Color = Color::srgb(0.5, 0.5, 1.0);
const SCORE_COLOR: Color = Color::srgb(1.0, 0.5, 0.5);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // .add_plugins(
        //     stepping::SteppingPlugin::default()
        //         .add_schedule(Update)
        //         .add_schedule(FixedUpdate)
        //         .at(Val::Percent(35.0), Val::Percent(50.0)),
        // )
        .insert_resource(Score(0))
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .add_event::<CollisionEvent>()
        .add_systems(Startup, setup)
        // Add our gameplay simulation systems to the fixed timestep schedule
        // which runs at 64 Hz by default
        .add_systems(
            FixedUpdate,
            (
                apply_velocity,
                move_paddle,
                check_for_collisions,
                play_collision_sound,
            )
                // `chain`ing systems together runs them in order
                .chain(),
        )
        .add_systems(Update, update_scoreboard)
        .run();
}

#[derive(Component)]
struct Paddle;

#[derive(Component)]
struct Ball;

#[derive(Component, Deref, DerefMut)]
struct Velocity(Vec2);

#[derive(Component)]
struct Collider;

#[derive(Event, Default)]
struct CollisionEvent;

#[derive(Component)]
struct Brick;

#[derive(Resource, Deref)]
struct CollisionSound(Handle<AudioSource>);

// This bundle is a collection of the components that define a "wall" in our game
#[derive(Bundle)]
struct WallBundle {
    // You can nest bundles inside of other bundles like this
    // Allowing you to compose their functionality
    sprite: Sprite,
    transform: Transform,
    collider: Collider,
}

/// Which side of the arena is this wall located on?
enum WallLocation {
    Left,
    Right,
    Bottom,
    Top,
}

impl WallLocation {
    /// Location of the *center* of the wall, used in `transform.translation()`
    fn position(&self) -> Vec2 {
        match self {
            WallLocation::Left => Vec2::new(LEFT_WALL, 0.),
            WallLocation::Right => Vec2::new(RIGHT_WALL, 0.),
            WallLocation::Bottom => Vec2::new(0., BOTTOM_WALL),
            WallLocation::Top => Vec2::new(0., TOP_WALL),
        }
    }

    /// (x, y) dimensions of the wall, used in `transform.scale()`
    fn size(&self) -> Vec2 {
        let arena_height = TOP_WALL - BOTTOM_WALL;
        let arena_width = RIGHT_WALL - LEFT_WALL;
        // Make sure we haven't messed up our constants
        assert!(arena_height > 0.0);
        assert!(arena_width > 0.0);

        match self {
            WallLocation::Left | WallLocation::Right => {
                Vec2::new(WALL_THICKNESS, arena_height + WALL_THICKNESS)
            }
            WallLocation::Bottom | WallLocation::Top => {
                Vec2::new(arena_width + WALL_THICKNESS, WALL_THICKNESS)
            }
        }
    }
}

impl WallBundle {
    // This "builder method" allows us to reuse logic across our wall entities,
    // making our code easier to read and less prone to bugs when we change the logic
    fn new(location: WallLocation) -> WallBundle {
        WallBundle {
            sprite: Sprite::from_color(WALL_COLOR, Vec2::ONE),
            transform: Transform {
                // We need to convert our Vec2 into a Vec3, by giving it a z-coordinate
                // This is used to determine the order of our sprites
                translation: location.position().extend(0.0),
                // The z-scale of 2D objects must always be 1.0,
                // or their ordering will be affected in surprising ways.
                // See https://github.com/bevyengine/bevy/issues/4149
                scale: location.size().extend(1.0),
                ..default()
            },
            collider: Collider,
        }
    }
}

// This resource tracks the game's score
#[derive(Resource, Deref, DerefMut)]
struct Score(usize);

#[derive(Component)]
struct ScoreboardUi;

// Add the game's entities to our world
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Camera
    commands.spawn(Camera2d);

    // Sound
    let ball_collision_sound = asset_server.load("sounds/breakout_collision.ogg");
    commands.insert_resource(CollisionSound(ball_collision_sound));

    // Paddle
    let paddle_y = BOTTOM_WALL + GAP_BETWEEN_PADDLE_AND_FLOOR;

    commands.spawn((
        Sprite::from_color(PADDLE_COLOR, Vec2::ONE),
        Transform {
            translation: Vec3::new(0.0, paddle_y, 0.0),
            scale: PADDLE_SIZE.extend(1.0),
            ..default()
        },
        Paddle,
        Collider,
    ));

    // Ball
    commands.spawn((
        Mesh2d(meshes.add(Circle::default())),
        MeshMaterial2d(materials.add(BALL_COLOR)),
        Transform::from_translation(BALL_STARTING_POSITION)
            .with_scale(Vec2::splat(BALL_DIAMETER).extend(1.)),
        Ball,
        Velocity(INITIAL_BALL_DIRECTION.normalize() * BALL_SPEED),
    ));

    // Scoreboard
    commands
        .spawn((
            Text::new("Score: "),
            TextFont {
                font_size: SCOREBOARD_FONT_SIZE,
                ..default()
            },
            TextColor(TEXT_COLOR),
            ScoreboardUi,
            Node {
                position_type: PositionType::Absolute,
                top: SCOREBOARD_TEXT_PADDING,
                left: SCOREBOARD_TEXT_PADDING,
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextFont {
                font_size: SCOREBOARD_FONT_SIZE,
                ..default()
            },
            TextColor(SCORE_COLOR),
        ));

    // Walls
    commands.spawn(WallBundle::new(WallLocation::Left));
    commands.spawn(WallBundle::new(WallLocation::Right));
    commands.spawn(WallBundle::new(WallLocation::Bottom));
    commands.spawn(WallBundle::new(WallLocation::Top));

    // Bricks
    let total_width_of_bricks = (RIGHT_WALL - LEFT_WALL) - 2. * GAP_BETWEEN_BRICKS_AND_SIDES;
    let bottom_edge_of_bricks = paddle_y + GAP_BETWEEN_PADDLE_AND_BRICKS;
    let total_height_of_bricks = TOP_WALL - bottom_edge_of_bricks - GAP_BETWEEN_BRICKS_AND_CEILING;

    assert!(total_width_of_bricks > 0.0);
    assert!(total_height_of_bricks > 0.0);

    // Given the space available, compute how many rows and columns of bricks we can fit
    let n_columns = (total_width_of_bricks / (BRICK_SIZE.x + GAP_BETWEEN_BRICKS)).floor() as usize;
    let n_rows = (total_height_of_bricks / (BRICK_SIZE.y + GAP_BETWEEN_BRICKS)).floor() as usize;
    let n_vertical_gaps = n_columns - 1;

    // Because we need to round the number of columns,
    // the space on the top and sides of the bricks only captures a lower bound, not an exact value
    let center_of_bricks = (LEFT_WALL + RIGHT_WALL) / 2.0;
    let left_edge_of_bricks = center_of_bricks
        // Space taken up by the bricks
        - (n_columns as f32 / 2.0 * BRICK_SIZE.x)
        // Space taken up by the gaps
        - n_vertical_gaps as f32 / 2.0 * GAP_BETWEEN_BRICKS;

    // In Bevy, the `translation` of an entity describes the center point,
    // not its bottom-left corner
    let offset_x = left_edge_of_bricks + BRICK_SIZE.x / 2.;
    let offset_y = bottom_edge_of_bricks + BRICK_SIZE.y / 2.;

    for row in 0..n_rows {
        for column in 0..n_columns {
            let brick_position = Vec2::new(
                offset_x + column as f32 * (BRICK_SIZE.x + GAP_BETWEEN_BRICKS),
                offset_y + row as f32 * (BRICK_SIZE.y + GAP_BETWEEN_BRICKS),
            );

            // brick
            commands.spawn((
                Sprite {
                    color: BRICK_COLOR,
                    ..default()
                },
                Transform {
                    translation: brick_position.extend(0.0),
                    scale: Vec3::new(BRICK_SIZE.x, BRICK_SIZE.y, 1.0),
                    ..default()
                },
                Brick,
                Collider,
            ));
        }
    }
}

fn move_paddle(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut paddle_transform: Single<&mut Transform, With<Paddle>>,
    time: Res<Time>,
) {
    let mut direction = 0.0;

    if keyboard_input.pressed(KeyCode::ArrowLeft) {
        direction -= 1.0;
    }

    if keyboard_input.pressed(KeyCode::ArrowRight) {
        direction += 1.0;
    }

    // Calculate the new horizontal paddle position based on player input
    let new_paddle_position =
        paddle_transform.translation.x + direction * PADDLE_SPEED * time.delta_secs();

    // Update the paddle position,
    // making sure it doesn't cause the paddle to leave the arena
    let left_bound = LEFT_WALL + WALL_THICKNESS / 2.0 + PADDLE_SIZE.x / 2.0 + PADDLE_PADDING;
    let right_bound = RIGHT_WALL - WALL_THICKNESS / 2.0 - PADDLE_SIZE.x / 2.0 - PADDLE_PADDING;

    paddle_transform.translation.x = new_paddle_position.clamp(left_bound, right_bound);
}

fn apply_velocity(mut query: Query<(&mut Transform, &Velocity)>, time: Res<Time>) {
    for (mut transform, velocity) in &mut query {
        transform.translation.x += velocity.x * time.delta_secs();
        transform.translation.y += velocity.y * time.delta_secs();
    }
}

fn update_scoreboard(
    score: Res<Score>,
    score_root: Single<Entity, (With<ScoreboardUi>, With<Text>)>,
    mut writer: TextUiWriter,
) {
    *writer.text(*score_root, 1) = score.to_string();
}

fn check_for_collisions(
    mut commands: Commands,
    mut score: ResMut<Score>,
    ball_query: Single<(&mut Velocity, &Transform), With<Ball>>,
    collider_query: Query<(Entity, &Transform, Option<&Brick>), With<Collider>>,
    mut collision_events: EventWriter<CollisionEvent>,
) {
    let (mut ball_velocity, ball_transform) = ball_query.into_inner();

    for (collider_entity, collider_transform, maybe_brick) in &collider_query {
        let collision = ball_collision(
            BoundingCircle::new(ball_transform.translation.truncate(), BALL_DIAMETER / 2.),
            Aabb2d::new(
                collider_transform.translation.truncate(),
                collider_transform.scale.truncate() / 2.,
            ),
        );

        if let Some(collision) = collision {
            // Sends a collision event so that other systems can react to the collision
            collision_events.send_default();

            // Bricks should be despawned and increment the scoreboard on collision
            if maybe_brick.is_some() {
                commands.entity(collider_entity).despawn();
                **score += 1;
            }

            // Reflect the ball's velocity when it collides
            let mut reflect_x = false;
            let mut reflect_y = false;

            // Reflect only if the velocity is in the opposite direction of the collision
            // This prevents the ball from getting stuck inside the bar
            match collision {
                Collision::Left => reflect_x = ball_velocity.x > 0.0,
                Collision::Right => reflect_x = ball_velocity.x < 0.0,
                Collision::Top => reflect_y = ball_velocity.y < 0.0,
                Collision::Bottom => reflect_y = ball_velocity.y > 0.0,
            }

            // Reflect velocity on the x-axis if we hit something on the x-axis
            if reflect_x {
                ball_velocity.x = -ball_velocity.x;
            }

            // Reflect velocity on the y-axis if we hit something on the y-axis
            if reflect_y {
                ball_velocity.y = -ball_velocity.y;
            }
        }
    }
}

fn play_collision_sound(
    mut commands: Commands,
    mut collision_events: EventReader<CollisionEvent>,
    sound: Res<CollisionSound>,
) {
    // Play a sound once per frame if a collision occurred.
    if !collision_events.is_empty() {
        // This prevents events staying active on the next frame.
        collision_events.clear();
        commands.spawn((AudioPlayer(sound.clone()), PlaybackSettings::DESPAWN));
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum Collision {
    Left,
    Right,
    Top,
    Bottom,
}

// Returns `Some` if `ball` collides with `bounding_box`.
// The returned `Collision` is the side of `bounding_box` that `ball` hit.
fn ball_collision(ball: BoundingCircle, bounding_box: Aabb2d) -> Option<Collision> {
    if !ball.intersects(&bounding_box) {
        return None;
    }

    let closest = bounding_box.closest_point(ball.center());
    let offset = ball.center() - closest;
    let side = if offset.x.abs() > offset.y.abs() {
        if offset.x < 0. {
            Collision::Left
        } else {
            Collision::Right
        }
    } else if offset.y > 0. {
        Collision::Top
    } else {
        Collision::Bottom
    };

    Some(side)
}
