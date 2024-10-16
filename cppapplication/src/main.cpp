#include "utils/main_util.hpp"

// Scene settings
int w = 112*2, h = 224*2;

// Camera settings
const double focal_length = 2.16;
const double field_of_view = 0.7854; // 45 degrees
const Eigen::Vector3d camera_position(0, 0, -100);

// Rotation settings
bool rotate = false;
double rX =-.05, rY =.4, rZ =.05;//Rotation IN RADIANS

// Lights
std::vector<Eigen::Vector3d> light_positions;
std::vector<Eigen::Vector4d> light_colors;

// Shader settings
const double diffuse_intensity = 0.4;
const double specular_intensity = 0.4;
const double ambient_light = 0.005;
const double shine = 32.0;
const double a = 1., b = .1, c = .01;//Attenuation constants

// Variant to store different objects
using Intersectable = std::variant<Triangle, Sphere, Mesh>;
std::vector<Intersectable> objects;
std::vector<Mesh> meshes;

Eigen::Vector3d compute_normal(const std::variant<Triangle, Sphere, Mesh> &obj, const Eigen::Vector3d &hit_point, const Triangle *hit_triangle = nullptr){
    return std::visit([&](const auto &shape) -> Eigen::Vector3d{
        if constexpr (std::is_same_v<decltype(shape), const Sphere&>) {//Sphere
            return (hit_point - shape.center).normalized();
        } else if constexpr (std::is_same_v<decltype(shape), const Triangle&>) {//Triangle
            return shape.normal();
        } else if constexpr (std::is_same_v<decltype(shape), const Mesh&>) {//Mesh
            if (hit_triangle != nullptr) {
                return hit_triangle->normal();  
            }
            return Eigen::Vector3d(0, 0, 0);  // No valid hit triangle, fallback
        }
    }, obj);
}

std::optional<std::tuple<double, Eigen::Vector3d, Intersectable, const Triangle *>> find_nearest_object(const Ray &ray){
    //Find the nearest intersecting mesh or sphere
    std::optional<std::tuple<double, Eigen::Vector3d, Intersectable, const Triangle *>> nearest_hit;
    double min_t = INFINITY;
    for (const auto &object : objects){
        auto hit = std::visit(Intersect(ray), object);//Find intersection based on this object type
        if (hit.has_value()){
            double t = std::get<0>(hit.value());
            if (t < min_t){
                min_t = t;
                const Triangle *hit_triangle = nullptr;
                if (std::holds_alternative<Mesh>(object)){//Mesh
                    const Mesh &mesh = std::get<Mesh>(object);
                    auto mesh_hit = mesh.intersects(ray);
                    if (mesh_hit.has_value()) hit_triangle = std::get<2>(mesh_hit.value()); // Get the hit triangle
                }
                // For Sphere, hit_triangle remains nullptr.
                nearest_hit = std::make_tuple(t, std::get<1>(hit.value()), object, hit_triangle);
            }
        }
    }
    return nearest_hit;
}

Eigen::Vector3d shoot_ray(const Ray &ray) {
    Eigen::Vector3d color(0., 0., 0.); 
    auto nearest_hit = find_nearest_object(ray);
    if (nearest_hit.has_value()) {
        double brightness = ambient_light;
        const auto &[t, hit_point, nearest_object, hit_triangle] = nearest_hit.value();
        Eigen::Vector3d N = compute_normal(nearest_object, hit_point, hit_triangle);
        N.normalize();
        Eigen::Vector3d V = -ray.direction;
        V.normalize();
        for (int i = 0; i < light_positions.size(); i++) {
            Eigen::Vector3d L = (light_positions[i] - hit_point);
            double d = L.norm();
            L.normalize();
            Eigen::Vector3d light_rgb = light_colors[i].head<3>();
            double attenuation = 1. / (a + b * d + c * d * d);
            // Diffuse shading
            double lambertian = N.dot(L);
            lambertian = fmax(lambertian, 0.0);
            brightness += attenuation * diffuse_intensity * lambertian * light_rgb.norm();
            // Specular shading
            Eigen::Vector3d R = (2.0 * N.dot(L) * N - L).normalized();
            double spec_angle = R.dot(V);
            spec_angle = fmax(spec_angle, 0.0);
            double specular = pow(spec_angle, shine);
            brightness += attenuation * specular_intensity * specular * light_rgb.norm();
        }
        brightness = fmin(brightness, 1.0);
        color = Eigen::Vector3d(brightness, brightness, brightness); 
    }
    return color;
}

void print_scene_in_ascii(const Eigen::MatrixXd &Color, int w, int h) {
    // ASCII characters for brightness levels
    const std::string brightness_chars = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
    const int l = brightness_chars.size() - 1;
    auto [first_line, last_line] = find_boundary(Color, w, h);
    for (int j = first_line; j >= last_line; --j) {
        for (int i = 0; i < w; ++i) {
            double brightness = Color(i, j);
            brightness = std::max(0.0, std::min(1.0, brightness)); 
            char c = brightness_chars[static_cast<int>(l * brightness)];
            std::cout << c;
        }
        std::cout << std::endl;
    }
}

void raytrace(int w, int h){
    Eigen::MatrixXd Color = Eigen::MatrixXd::Zero(w, h); 
    const double aspect_ratio = double(w) / double(h);
    const double y = (((focal_length)*sin(field_of_view / 2)) / sin((180 - (90 + ((field_of_view * (180 / M_PI) / 2)))) * (M_PI / 180)));
    const double x = (y * aspect_ratio);
    Eigen::Vector3d image_origin(-x, y, camera_position[2] - focal_length);
    Eigen::Vector3d x_displacement(2.0 / w * x, 0, 0);
    Eigen::Vector3d y_displacement(0, -2.0 / h * y, 0);
    for (unsigned i = 0; i < w; ++i){
        for (unsigned j = 0; j < h; ++j){
            Eigen::Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;
            Ray r(camera_position, (camera_position - pixel_center).normalized());
            Eigen::Vector3d result = shoot_ray(r);
            if (result.size() > 0){
                Color(i, j) = result(0);
            }
            else{
                std::cerr << "Invalid ray result." << std::endl;
            }
        }
    }
    print_scene_in_ascii(Color, w, h);
}

void setup_scene(int argc, char* argv[]){ 
    load_meshes(argc,argv,meshes);
    double rX =-.05, rY =.4, rZ =.05;//Rotation IN RADIANS
    if(meshes.size() > 0 && rotate) meshes[0].triangles = rotate_mesh(meshes[0],rX,rY,rZ);//Rotate mesh 1
    //meshes[0].triangles = translate_mesh(meshes[0],5,5,5);//Translate mesh 1
    for (auto &mesh : meshes) objects.emplace_back(mesh);

    //Sphere example
    //Eigen::Vector3d sphere_center(0, 0, 1);               
    //objects.emplace_back(Sphere(sphere_center, 1.)); 
               
    light_colors.emplace_back(0.8, 0.8, 0.8, 1);//Light 1
    light_positions.emplace_back(0, 5, -30);  
    light_colors.emplace_back(0.4, 0.4, 0.4, 1);//Light 2
    light_positions.emplace_back(10, -5, -20);  
    light_colors.emplace_back(0.3, 0.3, 0.3, 1);//Light 3  
    light_positions.emplace_back(10, 5, 20);  
    light_colors.emplace_back(0.2, 0.2, 0.2, 1);//Light 4  
    light_positions.emplace_back(-10, 20, -30); 
}

int main(int argc, char* argv[]){
    auto start=std::chrono::high_resolution_clock::now();
    setup_scene(argc,argv);
    raytrace(w, h);
    auto end=std::chrono::high_resolution_clock::now();
    std::cout<<"Runtime: "<<std::chrono::duration<double>(end-start).count()<<" seconds"<<std::endl;
    return 0;
}
