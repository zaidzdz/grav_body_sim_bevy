use bevy::dev_tools::infinite_grid::{InfiniteGrid, InfiniteGridPlugin, InfiniteGridSettings};

use bevy::prelude::*;

use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use rand;
#[derive(Component)]
struct Mass(f32);
#[derive(Component)]
struct Velocity(Vec3);
#[derive(Component)]
struct Acceleration(Vec3);
#[derive(Component)]
struct DebugPath(Vec<Vec3>); //holds point for debug path
#[derive(Resource)]
struct UISettings {
    gravity_constant: f32,
    paused: bool,
    show_debug_settings: bool,
    debug_show_arrows: bool,
    debug_show_path: bool,
    spawn_position: Vec3,
    spawn_velocity: Vec3,
    spawn_mass: f32,
}
const MAX_DEBUG_PATH_LENGTH: usize = 500;
impl Default for UISettings {
    fn default() -> Self {
        Self {
            gravity_constant: 10.0,
            paused: false,
            debug_show_path: false,
            debug_show_arrows: false,
            show_debug_settings: false,
            spawn_position: Vec3::ZERO,
            spawn_velocity: Vec3::ZERO,
            spawn_mass: 1.0,
        }
    }
}

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(PanOrbitCameraPlugin); // simple camera stuff
    app.add_plugins(InfiniteGridPlugin);
    app.add_plugins(EguiPlugin::default());
    app.init_resource::<UISettings>();
    app.add_systems(Startup, setup);
    app.add_systems(EguiPrimaryContextPass, debug_panel);
    app.add_systems(Startup, setup_gizmo_config);
    app.add_systems(
        Update,
        (update_acc_bodies, draw_debug_stuff, integrate).chain(),
    );

    app.run();
}
fn debug_panel(
    mut contexts: EguiContexts,
    mut settings: ResMut<UISettings>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;

    let mut root_ui = egui::Ui::new(
        ctx.clone(),
        "root".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );

    egui::Panel::left("left_panel")
        .resizable(false)
        .show(&mut root_ui, |ui| {
            ui.add(
                egui::Slider::new(&mut settings.gravity_constant, 0.0..=50.0)
                    .text("Gravity Constant"),
            );
            ui.add(egui::Checkbox::new(&mut settings.paused, "Paused"));
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Position");
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_position.x)
                        .speed(1)
                        .prefix("X: "),
                );
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_position.y)
                        .speed(1)
                        .prefix("Y: "),
                );
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_position.z)
                        .speed(1)
                        .prefix("Z: "),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Velocity");
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_velocity.x)
                        .speed(1)
                        .prefix("X: "),
                );
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_velocity.y)
                        .speed(1)
                        .prefix("Y: "),
                );
                ui.add(
                    egui::DragValue::new(&mut settings.spawn_velocity.z)
                        .speed(1)
                        .prefix("Z: "),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Mass");
                ui.add(egui::DragValue::new(&mut settings.spawn_mass).speed(1));
            });
            if ui.add(egui::Button::new("Spawn Body")).clicked() {
                spawn_body(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    settings.spawn_position,
                    settings.spawn_velocity,
                    settings.spawn_mass,
                );
            };

            ui.separator();
            ui.add(egui::Checkbox::new(
                &mut settings.show_debug_settings,
                "Debug",
            ));
            if settings.show_debug_settings {
                ui.indent("debug_options", |ui| {
                    ui.checkbox(&mut settings.debug_show_arrows, "Show arrows");
                    ui.checkbox(&mut settings.debug_show_path, "Show path");
                });
            }
        });

    Ok(())
}
fn update_acc_bodies(
    mut query: Query<(&Transform, &mut Acceleration, &Mass)>,
    settings: Res<UISettings>,
) {
    if !settings.paused {
        //acc only has to be mut because we can integrate rest
        let mut pairs = query.iter_combinations_mut();
        while let Some([(transform1, mut acc1, mass1), (transform2, mut acc2, mass2)]) =
            pairs.fetch_next()
        {
            let delta = transform2.translation - transform1.translation;
            let dir = delta.normalize_or_zero();
            let force =
                settings.gravity_constant * ((mass1.0 * mass2.0) / delta.length_squared()) * dir;

            acc1.0 += force / mass1.0;
            acc2.0 -= force / mass2.0;
        }
    }
}
fn integrate(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut Acceleration, &mut Velocity)>,
    settings: Res<UISettings>,
) {
    if !settings.paused {
        for (mut transform, mut acc, mut velocity) in query.iter_mut() {
            velocity.0 += acc.0 * time.delta_secs();
            transform.translation += velocity.0 * time.delta_secs();
            acc.0 = Vec3::ZERO;
        }
    }
}
fn draw_cone_arrow(gizmos: &mut Gizmos, start: Vec3, end: Vec3, color: Color) {
    gizmos.line(start, end, color);

    gizmos
        .primitive_3d(
            &Cone {
                radius: 0.1,
                height: 0.6,
            },
            Isometry3d::new(
                end,
                Quat::from_rotation_arc((start - end).normalize_or_zero(), Vec3::Y),
            ),
            color,
        )
        .resolution(32);
}
fn construct_path_curve(points: &Vec<Vec3>) -> CubicCurve<Vec3> {
    CubicCardinalSpline::new_catmull_rom(points.iter().map(|p| *p))
        .to_curve()
        .unwrap()
}

fn setup_gizmo_config(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 0.25;
}
fn mass_to_radius(mass: f32) -> f32 {
    mass.cbrt() * 0.3
}

fn draw_debug_stuff(
    mut gizmos: Gizmos,
    mut query: Query<
        (&Acceleration, &Velocity, &Transform, &Mass, &mut DebugPath),
        Without<PanOrbitCamera>,
    >,
    settings: Res<UISettings>,
) {
    if (settings.show_debug_settings) {
        for (acceleration, velocity, transform, mass, mut debug_path) in query.iter_mut() {
            let radius = mass_to_radius(mass.0);

            debug_path.0.push(transform.translation);
            if debug_path.0.len() > MAX_DEBUG_PATH_LENGTH {
                //makes curve not last forever
                debug_path.0.remove(0);
            }

            if settings.debug_show_path {
                gizmos.linestrip(
                    construct_path_curve(&debug_path.0).iter_positions(300),
                    Color::srgb(1.0, 1.0, 1.0),
                )
            }
            if settings.debug_show_arrows {
                let start = transform.translation;
                let radius = mass_to_radius(mass.0);

                draw_cone_arrow(
                    &mut gizmos,
                    start,
                    start + (velocity.0 / 3.0) * radius,
                    Color::srgb(0.2, 1.0, 0.3),
                );
                draw_cone_arrow(
                    &mut gizmos,
                    start,
                    start + (acceleration.0 / 3.0) * radius,
                    Color::srgb(1.0, 0.2, 0.2),
                );
            }
        }
    }
}
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: Gizmos,
) {
    //spawn the camera
    commands.spawn((
        Transform::from_translation(Vec3::new(0.0, 1.5, 5.0)),
        PanOrbitCamera::default(),
    ));
    commands.spawn((
        InfiniteGrid,
        InfiniteGridSettings {
            x_axis_color: Color::srgb(0.2, 0.2, 0.2),
            z_axis_color: Color::srgb(0.2, 0.2, 0.2),
            ..default()
        },
    ));
    //spawn a light
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            shadow_maps_enabled: false,

            ..default()
        },
        Transform::from_xyz(0.0, 1.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    spawn_body(
        &mut commands,
        &mut meshes,
        &mut materials,
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 0.0),
        500.0,
    );
    spawn_body(
        &mut commands,
        &mut meshes,
        &mut materials,
        vec3(30.0, 0.0, 0.0),
        vec3(0.0, 0.0, -10.0),
        10.0,
    );
}
fn get_random_srgb() -> Color {
    Color::srgb(
        rand::random_range(0.0..1.0),
        rand::random_range(0.0..1.0),
        rand::random_range(0.0..1.0),
    )
}
fn spawn_body(
    mut commands: &mut Commands,
    mut meshes: &mut ResMut<Assets<Mesh>>,
    mut materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
    velocity: Vec3,
    mass: f32,
) {
    let body = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(mass_to_radius(mass)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: get_random_srgb(),
            reflectance: 0.0,
            perceptual_roughness: 1.0,
            ..default()
        })),
        Transform::from_translation(position),
        Mass(mass),
        Velocity(velocity),
        Acceleration(Vec3::new(0.0, 0.0, 0.0)),
        DebugPath(Vec::new()),
    ));
}
