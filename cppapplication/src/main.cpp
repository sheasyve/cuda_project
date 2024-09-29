#include "util.hpp"

using namespace Eigen;

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

//Material settings
const double reflection_intensity = 0.7;
const double ambient_intensity = 0.5;
const double diffuse_intensity = 0.5;
const double specular_intensity = 0.2;
const double specular_exponent = 256.0;

struct grayscale {
    MatrixXd &Intensity;
    MatrixXd &A;
};

struct grayscale get_grayscale_matrixes(int w, int h) {
    MatrixXd Intensity = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h);  
    return {Intensity, A};
}

void raytrace(int w, int h) {
    struct grayscale shading = get_grayscale_matrixes(w, h);
    const double aspect_ratio = double(w) / double(h);
    double image_y = (((focal_length)*sin(field_of_view / 2)) / sin((180 - (90 + ((field_of_view * (180 / M_PI) / 2)))) * (M_PI / 180)));
    double image_x = (image_y * aspect_ratio);
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);
    for (unsigned i = 0; i < w; ++i){
        for (unsigned j = 0; j < h; ++j){//Loop through every pixel
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;
            //Do raytracing here for this ray
        }
    }

}

int main() {
    int w= 200, h = 200;
    raytrace(w, h);
    
    return 0;
}
