#include "mainwindow.h"
#include <ui_mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->groupBox->hide();
    ui->groupBox_2->hide();

    ui->doubleSpinBox->setMaximum(1.0);
    ui->doubleSpinBox->setMaximum(1.0);

    ui->doubleSpinBox_4->setMaximum(1.0);
    ui->doubleSpinBox_4->setMaximum(1.0);

    ui->spinBox->setRange(0,255);
    ui->spinBox_2->setRange(0,255);
    ui->spinBox_3->setRange(0,255);

    ui->spinBox_5->setRange(-100,100);
    ui->spinBox_6->setRange(-100,100);
    ui->spinBox_7->setRange(-100,100);

    ui->label->setPixmap(QPixmap::fromImage(*paper));
}

MainWindow::~MainWindow()
{
    delete ui;
}

#include <fstream>
#include <cmath>

struct Scene;
struct Object;
struct HitInfo;
struct Color;
struct Raytracer;
void clamp255(Color& col);

struct Vec3
{
    double x,y,z;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator + (const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator - (const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator - () const { return Vec3(-x, -y, -z); }
    Vec3 operator * (double d) const { return Vec3(x*d, y*d, z*d); }
    Vec3 operator / (double d) const { return Vec3(x/d, y/d, z/d); }

    double lenght() const
    {
      return sqrt(x*x + y*y + z*z);
    }
    Vec3 normalize() const
    {
    double mg = lenght();
    return Vec3(x/mg,y/mg,z/mg);
    }
    double lenghtPow() const
    {
      return (x*x+y*y+z*z);
    }

    Vec3 cross(Vec3 v)
    {
    return Vec3((y * v.z) - (z * v.y),
               (z * v.x) - (x * v.z),
               (x * v.y) - (y * v.x));
    }

};

inline double dot(const Vec3& a, const Vec3& b)
{
    return (a.x*b.x + a.y*b.y + a.z*b.z);
}

Vec3 reflect(Vec3 vec, Vec3 normal)
{
    double d_dot = dot(normal,vec);
    return normal * d_dot * 2 - vec;
}

const Vec3 emptyVec3 = Vec3(0,0,0);

struct Vec2
{
    double x,y;
    Vec2() : x(0), y(0) {}
    Vec2(double x, double y) : x(x), y(y) {}
    Vec2 operator + (const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
    Vec2 operator - (const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
    Vec2 operator * (double d) const { return Vec2(x*d, y*d); }
    Vec2 operator / (double d) const { return Vec2(x/d, y/d); }
};

struct Ray
{
    Vec3 o,d;
    double epsilon = 0.000001;
    double max = 1.0/0.0;
    Ray(const Vec3& o, const Vec3& d) : o(o), d(d.normalize()) {}
};

struct Color
{
    double red;
    double green;
    double blue;
    Color(double red, double green, double blue) : red(red), green(green), blue(blue){}
    Color operator + (Color d){ return Color(red+d.red,green+d.green,blue+d.blue);}
    Color operator * (Color d){ return Color(red*d.red,green*d.green,blue*d.blue);}
    Color operator * (double t){ return Color(red*t,green*t,blue*t);}
    Color operator / (double t){ return Color(red/t,green/t,blue/t);}
    double getRed(){ return red*255; }
    double getGreen(){ return green*255; }
    double getBlue(){ return blue*255; }
};

const Color white = Color(255/255.0,255/255.0,255/255.0);
const Color black = Color(0,0,0);
const Color red = Color(255/255.0,0,0);
const Color green = Color(0,255/255.0,0);
const Color blue = Color(0,0,255/255.0);
const Color yellow = Color(255/255.0,255/255.0,0);
const Color cyan = Color(0,255/255.0,255/255.0);
const Color magenta = Color(255/255.0,0,255/255.0);
const Color gray = Color(192/255.0,192/255.0,192/255.0);
const Color orange = Color(255/255.0,165/255.0,0);

struct ICamera
{
    virtual Ray GetRayTo(Vec2 relativeLocation) = 0;
    virtual ~ICamera(){}
};

struct OrthonormalBasis
{
    Vec3 u = emptyVec3;
    Vec3 v = emptyVec3;
    Vec3 w = emptyVec3;
    OrthonormalBasis(Vec3 front, Vec3 up)
    {
        w = front.normalize();
        u = up.cross(w);
        u = u.normalize();
        v = w.cross(u);
    }
};

inline Vec3 operator * (OrthonormalBasis onb, Vec3 v)
{
    return (onb.u * v.x + onb.v * v.y + onb.w * v.z);
}

struct Orthogonal : public ICamera
{
    Vec3 EyePosition;
    double Angle;
    Vec2 CameraSize;
    Orthogonal(Vec3 eye, double angle, Vec2 size) :
              EyePosition(eye),
              Angle(angle),
              CameraSize(size){}

    Ray GetRayTo(Vec2 pictureLocation)
    {
        Vec3 direction = Vec3(sin(Angle),0,cos(Angle));
        direction = direction.normalize();
        Vec2 offsetFromCenter = Vec2(pictureLocation.x * CameraSize.x,pictureLocation.y * CameraSize.y);
        Vec3 position = Vec3(
            EyePosition.x + offsetFromCenter.x * cos(Angle),
            EyePosition.y + offsetFromCenter.y,
            EyePosition.z + offsetFromCenter.x * sin(Angle));
        return Ray(position,direction);
    }
};

struct Pinhole : public ICamera
{
    OrthonormalBasis onb = OrthonormalBasis(emptyVec3,emptyVec3);
    Vec3 origin = emptyVec3;
    double distance;
    Vec2 scale;
    Pinhole(Vec3 origin, Vec3 lookAt,Vec3 up, Vec2 scale, double distance)
    {
        this->onb = OrthonormalBasis(origin - lookAt, up);
        this->origin = origin;
        this->distance = distance;
        this->scale = scale;
    }
    Ray GetRayTo(Vec2 relativeLocation)
    {
        Vec2 vpLoc = Vec2(relativeLocation.x * scale.x, relativeLocation.y * scale.y);
        return Ray(origin, RayDirection(vpLoc));
    }
    Vec3 RayDirection(Vec2 v)
    {
        return onb * Vec3(v.x, v.y, -distance);
    }
};

struct HitInfo
{
    Object* hitObject;
    Scene* scene;
    int depth; // max ilosc odbic
    Vec3 normal = emptyVec3;
    Vec3 hitPoint = emptyVec3;
    Ray ray = Ray(emptyVec3,emptyVec3);
};

struct IMaterial
{
    virtual Color shade(Raytracer* tracer, HitInfo hit) = 0;
};


struct ISampleGenerator
{
    virtual QVector<Vec2> Sample(int count) = 0;
};

struct ISampleDistributor
{
    virtual Vec2 MapSample(Vec2 sample) = 0;
};

struct PureRandom : ISampleGenerator
{
    unsigned int r;
    double R_MAX = 1.0/0.0;

    PureRandom(unsigned int seed)
    {
        qsrand(seed);
    }

    QVector<Vec2> Sample (int sampleCt)
    {
        QVector<Vec2> samples = QVector<Vec2>(sampleCt);
        for (int i = 0; i < sampleCt; i++)
        {
            samples[i] = Vec2(qrand()/double(RAND_MAX), qrand()/double(RAND_MAX));
        }
        return samples;
    }
};

struct Regular : ISampleGenerator
{
    QVector<Vec2> Sample(int sampleCt)
    {
        int sampleRow = int(qPow(sampleCt,0.5));
        QVector<Vec2> result = QVector<Vec2>(sampleRow * sampleRow);
        for (int x = 0; x < sampleRow; x++)
        {
            for (int y = 0; y < sampleRow; y++)
            {
                double fracX = (x + 0.5) / sampleRow;
                double fracY = (y + 0.5) / sampleRow;
                result[x * sampleRow + y] = Vec2(fracX, fracY);
            }
        }
        return result;
    }
};

struct Jittered : ISampleGenerator
{
    unsigned int r;
    double R_MAX = 1.0/0.0;

    Jittered(unsigned int seed)
    {
        qsrand(seed);
    }

    QVector<Vec2> Sample(int count)
    {
        int sampleRow = int(qPow(count,0.5));
        QVector<Vec2> result = QVector<Vec2>(sampleRow * sampleRow);
        for (int x = 0; x < sampleRow; x++)
        {
            for (int y = 0; y < sampleRow; y++)
            {
                double fracX = (x + qrand()/double(RAND_MAX)) / sampleRow / double(qPow(2,x));
                double fracY = (y + qrand()/double(RAND_MAX)) / sampleRow / double(qPow(2,x));
                result[x * sampleRow + y] = Vec2(fracX, fracY);
            }
        }
        return result;
    }
};

struct NRooks : ISampleGenerator
{
    unsigned int r;
    double R_MAX = 1.0/0.0;

    NRooks(unsigned int seed)
    {
        qsrand(seed);
    }

    QVector<Vec2> Sample(int sampleCt)
    {
        QVector<Vec2> samples = QVector<Vec2>(sampleCt);
        for (int i = 0; i < sampleCt; i++)
        {
            samples[i] = Vec2((i + qrand()/double(RAND_MAX)) / sampleCt,
                              (i + qrand()/double(RAND_MAX)) / sampleCt);
        }

        ShuffleX(samples, sampleCt);
        return samples;
    }

    void ShuffleX(QVector<Vec2> samples, int sampleCt)
    {
        for (int i = 0; i < sampleCt - 1; i++)
        {
            int target = qrand() % sampleCt;
            double temp = samples[i].x;
            samples[i].x = samples[target].x;
            samples[target].x = temp;
        }
    }
};

struct SquareDistributor : ISampleDistributor
{
    Vec2 MapSample(Vec2 sample)
    {
        return sample;
    }
};

struct DiskDistributor : ISampleDistributor
{
    Vec2 MapSample(Vec2 sample)
    {
        sample.x = sample.x * 2 - 1;
        sample.y = sample.y * 2 - 1;
        double r;
        double phi;
        if (sample.x > -sample.y)
        {
            if (sample.x > sample.y)
            {
                r = sample.x;
                phi = sample.y / sample.x;
            }
            else
            {
                r = sample.y;
                phi = 2 - sample.x / sample.y;
            }
        }
        else
        {
            if (sample.x < sample.y)
            {
                r = -sample.x;
                phi = 4 + sample.y / sample.x;
            }
            else
            {
                r = -sample.y;
                phi = 6 - sample.x / sample.y;
            }
         }
         if (sample.x == 0.0 && sample.y == 0.0)
         {
             phi = 0;
         }

         phi *= M_PI / 4;
         return Vec2( r * qCos(phi) , r * qSin(phi) );
    }
};

struct Sampler
{
    QVector< QVector<Vec2> > sets;
    int sampleNdx = 0;
    int setNdx = 1;
    int SampleCount;
    int SetCount;

    Sampler(){}

    Sampler(ISampleGenerator* sampler,
            ISampleDistributor* mapper,
            int sampleCt,
            int setCt)
    {
        this->sets = QVector< QVector<Vec2> >(setCt);
        this->SampleCount = sampleCt;
        for (int i = 0; i < setCt; i++)
        {
            QVector<Vec2> samples = sampler->Sample(sampleCt);
            QVector<Vec2> bufor;
            for(int x = 0; x < samples.size(); x++)
            {
                bufor.push_back(mapper->MapSample(samples[x]));
            }
            this->sets[i] = bufor;
        }
    }

    Vec2 Single()
    {
        Vec2 sample = sets[setNdx][sampleNdx];
        sampleNdx++;
        if (sampleNdx >= sets[setNdx].length())
        {
            sampleNdx = 0;
            setNdx = rand() % sets.size();
        }
        return sample;
    }
};

struct Light
{
    Vec3 center = emptyVec3;
    double radius;
    Sampler sampler;
    Color color = black;
    // Konstruktor udający światło punktowe
    Light(Vec3 position, Color color)
    {
        this->center = position;
        this->color = color;
        this->radius = 0;
    }

    // pełny konstruktor
    Light(Vec3 center, Color color, Sampler sampler, double radius)
    {
        this->center = center;
        this->radius = radius;
        this->sampler = sampler;
        this->color = color;
    }

    Vec3 Sample()
    {
        if (radius == 0.0)
        {
            return center;
        } // symulowanie światła punktowego

        return center + RemapSampleToUnitSphere(sampler.Single()) * radius;
    }

    Vec3 RemapSampleToUnitSphere(Vec2 sample)
    {
        double z = 2 * sample.x - 1;
        double t = 2 * M_PI * sample.y;
        double r = qPow(1 - z * z, 0.5);
        return Vec3(r * qCos(t),r * qSin(t),z);
    }

    Color getColor()
    {
        return color;
    }
};

struct Object
{
    Object(IMaterial* material):material(material){}
    IMaterial* material;
    virtual IMaterial* getMaterial() = 0;
    virtual bool intersect(const Ray& ray, double &t, Vec3& normal) = 0;
    virtual ~Object(){}
};

struct Scene
{
    Color background;
    QVector <Object*> objects;
    QVector <Light*> lights;

    Scene(Color background = black) : background(background) {}

    void addObj(Object* obj)
    {
        objects.push_back(obj);
    }

    void addLight(Light* light)
    {
        lights.push_back(light);
    }

    HitInfo traceRay(const Ray& ray)
    {
        HitInfo result = HitInfo();
        Vec3 normal = emptyVec3;
        double minimalDistance = ray.max; // najbliższe trafienie
        double hitDistance = 0; // zmienna pomocnicza, ostatnia odległość

        for( int i = 0; i < objects.size(); ++i)
        {
            if (objects[i]->intersect(ray, hitDistance, normal) && hitDistance < minimalDistance) // jeśli najbliższe trafienie
            {
                minimalDistance = hitDistance; // nowa najmniejsza odległość
                result.hitObject = objects[i]; // zapisz kolor trafionego obiektu
                result.normal = normal;
            }
        }

        if(result.hitObject != NULL)
        {
            result.hitPoint = ray.o + ray.d * minimalDistance;
            result.ray = ray;
            result.scene = this;
        }

        return result;
    }

    bool anyObstacleBetween(Vec3 pointA, Vec3 pointB)
    {
     // odległość od cieniowanego punktu do światła
     Vec3 vectorAB = pointB - pointA;
     double distAB = vectorAB.lenght();
     double currDistance = 1.0/0.0;
     // promień (półprosta) z cieniowanego punktu w kierunku światła
     Ray ray = Ray(pointA, vectorAB);

     Vec3 ignored = emptyVec3;
     for(int i = 0; i < objects.size(); ++i)
     {
     // jeśli jakiś obiekt jest na drodze promienia oraz trafienie
     // nastąpiło bliżej niż odległość punktu do światła,
     // obiekt jest w cieniu
        if ((objects[i]->intersect(ray, currDistance, ignored)) && (currDistance < distAB))
        {
            return true;
        }
     }
     // obiekt nie jest w cieniu
     return false;
    }
};

struct Raytracer
{
    int maxDepth;

    Raytracer(int maxDepth)
    {
        this->maxDepth = maxDepth;
    }

    void raytrace(Scene* scene, ICamera* camera, int w, int h, QImage* paper)
    {
        Sampler singleSample = Sampler(new Regular(), new SquareDistributor(), 1, 1);
        this->raytrace(scene, camera, w, h, paper, singleSample);
        return;
    }

    void raytrace(Scene* scene, ICamera* camera, int w, int h, QImage* paper, Sampler sampler)
    {
        uchar* ptr;
        for(int y = 0; y < h; y++)
        {
            ptr = paper->scanLine(y);
            for (int x = 0; x < w; x++)
            {
                Color totalColor = black;
                for (int i = 0; i < sampler.SampleCount; i++)
                {
                // przeskalowanie x i y do zakresu [-1; 1]
                    Vec2 sample = sampler.Single(); // pobierz próbkę
                    Vec2 pictureCoordinates = Vec2( // oblicz kierunek
                     ((x + sample.x) / (double)w) * 2 - 1,
                     ((y + sample.y) / (double)h) * 2 - 1);
                    Ray ray = camera->GetRayTo(pictureCoordinates);
                    HitInfo info = scene->traceRay(ray);
                    // ustawienie odpowiedniego koloru w obrazie wynikowym
                    Color color = black;
                    if (info.hitObject)
                    {
                        color = shadeRay(scene, ray, 0);
                        clamp255(color);
                        totalColor = totalColor + (color / (double)sampler.SampleCount);
                    }
                    else
                    {
                        color = scene->background ;
                        totalColor = totalColor + (color / (double)sampler.SampleCount);
                    }
                }

                clamp255(totalColor);
                ptr[4*x] = uchar(totalColor.getBlue());      //b
                ptr[4*x+1] = uchar(totalColor.getGreen());   //g
                ptr[4*x+2] = uchar(totalColor.getRed());     //r
            }

            if(y % 4 == 0)
            {
                qDebug() << (y/double(h))*100.0 << "%";
            }

        }
    }

    Color shadeRay(Scene* scene, Ray ray, int currentDepth)
    {
        if(currentDepth > maxDepth)
        {
            return black;
        }
        HitInfo info = scene->traceRay(ray);
        info.depth = currentDepth + 1;

        if (info.hitObject == NULL)
        {
            return scene->background;
        }
        IMaterial* material = info.hitObject->material;

        return material->shade(this, info);
    }
};

struct PerfectDiffuse : IMaterial
{
    Color materialColor = black;
    PerfectDiffuse(Color materialColor)
    {
        this->materialColor = materialColor;
    }

    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Color totalColor = black;
        for(int i = 0; i < hit.scene->lights.size(); ++i)
        {
            Vec3 lightPos = hit.scene->lights[i]->Sample();
            Vec3 inDirection = (lightPos - hit.hitPoint).normalize();
            double diffuseFactor = dot(inDirection,hit.normal);
            if (diffuseFactor < 0) { continue; }
            if (hit.scene->anyObstacleBetween(hit.hitPoint, lightPos))
            {
                continue;
            }
            totalColor = totalColor + hit.scene->lights[i]->color * materialColor * diffuseFactor;
        }
        return totalColor;
    }

};

struct Phong : public IMaterial
{
    Color materialColor = black;
    double diffuseCoeff;
    double specular;
    double specularExponent;
    Phong(Color materialColor,double diffuse,double specular,double specularExponent)
    {
        this->materialColor = materialColor;
        this->diffuseCoeff = diffuse;
        this->specular = specular;
        this->specularExponent = specularExponent;
    }

    Color radiance(Light light, HitInfo hit)
    {
        Vec3 lightPos = light.Sample();
        Vec3 inDirection = (lightPos - hit.hitPoint).normalize();
        double diffuseFactor = dot(inDirection,hit.normal);
        if (diffuseFactor < 0)
        {
            return black;
        }
        Color result = light.color * materialColor * diffuseFactor * diffuseCoeff;
        double phongFactor = PhongFactor(inDirection, hit.normal, -hit.ray.d);
        if (phongFactor != 0.0)
        {
            result = result + materialColor * specular * phongFactor;
        }
        return result;
    }


    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Color totalColor = black;
        for(int i = 0; i < hit.scene->lights.size(); ++i)
        {
            Vec3 lightPos = hit.scene->lights[i]->Sample();
            Vec3 inDirection = (lightPos - hit.hitPoint).normalize();
            double diffuseFactor = dot(inDirection,hit.normal);
            if (diffuseFactor < 0)
            {
                continue;
            }
            if (hit.scene->anyObstacleBetween(hit.hitPoint, lightPos))
            {
                continue;
            }

            Color result = hit.scene->lights[i]->color * materialColor * diffuseFactor * diffuseCoeff;
            double phongFactor = PhongFactor(inDirection, hit.normal, -hit.ray.d);
            if (phongFactor != 0.0)
            {
                result = result + materialColor * specular * phongFactor;
            }
            totalColor = totalColor + result;
        }
        return totalColor;
    }


    double PhongFactor(Vec3 inDirection, Vec3 normal, Vec3 toCameraDirection)
    {
        Vec3 reflected = reflect(inDirection, normal);
        double cosAngle = dot(reflected,toCameraDirection);
        if (cosAngle <= 0)
        {
            return 0;
        }
        return pow(cosAngle, specularExponent);
    }

};

struct Reflective : IMaterial
{
    Phong direct = Phong(black,0,0,0); // do bezpośredniego oświetlenia
    double reflectivity;
    Color reflectionColor = black;
    // diffuse - jak mocno rozprasza, specular - polysk, exponent - potega cos, reflectivity - jak mocno odpija swiatko ( =0 => Phong) (ref = 1, diff = 0 => lustro)
    // diffuse + reflectivity <= 1
    Reflective(Color materialColor , double diffuse , double specular , double exponent , double reflectivity)
    {
        this->direct = Phong(materialColor, diffuse, specular, exponent);
        this->reflectivity = reflectivity;
        this->reflectionColor = materialColor;
    }
    Color shade(Raytracer* tracer, HitInfo hit)
    {
        Vec3 toCameraDirection = -hit.ray.d;

        Color radiance = direct.shade(tracer, hit);
        Vec3 reflectionDirection = reflect(toCameraDirection, hit.normal);
        Ray reflectedRay = Ray(hit.hitPoint, reflectionDirection);
        radiance = radiance + tracer->shadeRay(hit.scene, reflectedRay, hit.depth) * reflectionColor * reflectivity;

        return radiance;
    }
};

IMaterial* whiteMir = new Reflective(white, 0.0, 1, 300, 1);
IMaterial* redMir = new Reflective(red, 0.0, 1, 300, 1);
IMaterial* greenMir = new Reflective(green, 0.0, 1, 300, 1);
IMaterial* blueMir = new Reflective(blue, 0.0, 1, 300, 1);
IMaterial* grayMir = new Reflective(gray, 0.0, 1, 300, 1);
IMaterial* yellowMir = new Reflective(yellow, 0.0, 1, 300, 1);
IMaterial* orangeMir = new Reflective(orange, 0.0, 1, 300, 1);

IMaterial* whiteMetal = new Phong(white, 0.8, 1, 20);
IMaterial* redMetal = new Phong(red, 0.8, 1, 20);
IMaterial* greenMetal = new Phong(green, 0.8, 1, 20);
IMaterial* blueMetal = new Phong(blue, 0.8, 1, 20);
IMaterial* grayMetal = new Phong(gray, 0.8, 1, 20);
IMaterial* yellowMetal = new Phong(yellow, 0.8, 1, 20);
IMaterial* orangeMetal = new Phong(orange, 0.8, 1, 20);


IMaterial* whiteMat = new Reflective(white, 0.4, 1, 300, 0.6);
IMaterial* redMat = new Reflective(red, 0.4, 1, 300, 0.6);
IMaterial* greenMat = new Reflective(green, 0.4, 1, 300, 0.6);
IMaterial* blueMat = new Reflective(blue, 0.4, 1, 300, 0.6);
IMaterial* grayMat = new Reflective(gray, 0.4, 1, 300, 0.6);
IMaterial* yellowMat = new Reflective(yellow, 0.4, 1, 300, 0.6);
IMaterial* orangeMat = new PerfectDiffuse(orange);

struct Sphere : public Object
{
    Vec3 center;
    double r;
    Sphere(Vec3 center, double r, IMaterial* material) : Object(material) , center(center), r(r){}
    bool intersect(const Ray& ray, double &t, Vec3& outNormal)
    {
        Vec3 o = ray.o;
        Vec3 d = ray.d;
        Vec3 oc = o - center; //distance
        const double a = d.lenghtPow();
        const double b = dot(oc, d) * 2;
        const double c = oc.lenghtPow() - r*r;
        double disc = b*b - 4 * c * a;
        if (disc < ray.epsilon)
        {
            return false;
        }
        disc = sqrt(disc);
        double t0 = (-b - disc)/(2*a);
        if(t0 < ray.epsilon)
        {
            t0 = (-b + disc)/(2*a);
            if(t0 < ray.epsilon)
            {
                return false;
            }
        }
        t = t0;

        Vec3 pi = ray.o + ray.d * t;
        outNormal = (pi - center).normalize();
        return true;
    }
    IMaterial* getMaterial()
    {
        return material;
    }
};

struct Plane : public Object
{
    /// <summary>Punkt przez który płaszczyzna przechodzi</summary>
    Vec3 point = emptyVec3;
    /// <summary>Normalna do płaszczyzny</summary>
    Vec3 normal = emptyVec3;

    Plane(Vec3 point, Vec3 normal, IMaterial* material) : Object(material)
    {
        this->point = point;
        this->normal = normal;
        this->material = material;
    }

    bool intersect(const Ray& ray, double& t, Vec3& outNormal)
    {
        double dis = dot((point - ray.o), normal.normalize()) / dot(ray.d, normal);
        if (dis > ray.epsilon)
        {
            t = dis;
            outNormal = normal;
            return true;
        }
        return false;
    }

    Vec3 getNormal()
    {
        return normal;
    }
    IMaterial* getMaterial()
    {
        return material;
    }
};


void clamp255(Color& col)
{
    col.red = (col.red > 1) ? 1 : (col.red < 0) ? 0 : col.red;
    col.green = (col.green > 1) ? 1 : (col.green < 0) ? 0 : col.green;
    col.blue = (col.blue > 1) ? 1 : (col.blue < 0) ? 0 : col.blue;
}


Scene* scene = new Scene(Color(0,100/255.0,170/255.0));

void MainWindow::raytracing()
{
    scene->addLight(new Light(Vec3(0, 5, -5), white));

    scene->addObj(new Plane(Vec3(0,-2,0),Vec3(0,1,0),grayMat));

    ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),Vec2(1,1.5),1);

    //const int SampleCt = 1;

    Raytracer* tracer = new Raytracer(5);

    //Sampler antiAlias = Sampler(new Regular(), new SquareDistributor(), SampleCt, 1); // na razie wystarczy jeden set

    tracer->raytrace(scene, camera, W, H, paper);

    ui->label->setPixmap(QPixmap::fromImage(*paper));
}

void MainWindow::on_pushButton_clicked()
{
    raytracing();
    update();
}

/* ------------- presets ----------------
 * Base
 * scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, blueMat));
 * scene->addObj(new Sphere(Vec3(4, 0, 0), 2, greenMetal));
 * scene->addObj(new Sphere(Vec3(0, 0, 3), 2, redMir));
 * scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMir));
 *
 * scene->addLight(new Light(Vec3(0, 5, -5), white));
 *
 * ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);
 *
 * Mirror
 * scene->addObj(new Sphere(Vec3(0, 0, 0), 9, new Reflective(white,0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, new Reflective(Color(51/255.0,1,1),0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(4, 0, 0), 2, new Reflective(Color(1,178/255.0,102/255.0),0,1,30,1)));
 * scene->addObj(new Sphere(Vec3(0, 0, 3), 2, new Reflective(Color(0,153/255.0,76/255.0),0,1,30,1)));
 * scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMir));
 *
 * scene->addLight(new Light(Vec3(0, 3, -3), white));
 * scene->addLight(new Light(Vec3(0, 3, 5), white));
 * scene->addLight(new Light(Vec3(3, 3, -3), white));
 * scene->addLight(new Light(Vec3(-3, 3, 5), white));
 *
 * ICamera* camera = new Pinhole(Vec3(0, 1, -5),Vec3(0, 0, 0),Vec3(0, -1, 0),1);
*/

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    if(index == 0)
    {
        ui->groupBox->hide();
        ui->groupBox_2->hide();
    }
    else if (index == 1)
    {
        ui->groupBox->show();
        ui->groupBox_2->hide();
    }
    else
    {
        ui->groupBox->show();
        ui->groupBox_2->show();
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    switch(ui->comboBox->currentIndex())
    {
        case 0:
        scene->addObj(new Sphere(Vec3(ui->spinBox_5->value(),
                                      ui->spinBox_6->value(),
                                      ui->spinBox_7->value()),
                                 ui->spinBox_4->value(),
                                 new PerfectDiffuse(Color(ui->spinBox->value() / 255.0,
                                                          ui->spinBox_2->value() / 255.0,
                                                          ui->spinBox_3->value() / 255.0 ))));
            break;
        case 1:
        scene->addObj(new Sphere(Vec3(ui->spinBox_5->value(),
                                      ui->spinBox_6->value(),
                                      ui->spinBox_7->value()),
                                 ui->spinBox_4->value(),
                                 new Phong(Color(ui->spinBox->value() / 255.0,
                                                 ui->spinBox_2->value() / 255.0,
                                                 ui->spinBox_3->value() / 255.0 ),
                                           ui->doubleSpinBox->value(),
                                           ui->doubleSpinBox_2->value(),
                                           ui->doubleSpinBox_3->value())));
            break;
        case 2:
        scene->addObj(new Sphere(Vec3(ui->spinBox_5->value(),
                                      ui->spinBox_6->value(),
                                      ui->spinBox_7->value()),
                                 ui->spinBox_4->value(),
                                 new Reflective(Color(ui->spinBox->value() / 255.0,
                                                      ui->spinBox_2->value() / 255.0,
                                                      ui->spinBox_3->value() / 255.0 ),
                                                ui->doubleSpinBox->value(),
                                                ui->doubleSpinBox_2->value(),
                                                ui->doubleSpinBox_3->value(),
                                                ui->doubleSpinBox_4->value())));
            break;
    }
}



void MainWindow::on_pushButton_3_clicked()
{
    scene = new Scene(Color(0,153,153));

    ICamera* camera = new Pinhole(Vec3(0, 5, -5),Vec3(0, 2, 0),Vec3(0, -1, 0),Vec2(1,1),1.0);

    Raytracer* tracer = new Raytracer(5);

    const int SampleCt = 16;

    qsrand(SampleCt);

    Sampler antiAlias = Sampler(new NRooks(0), new SquareDistributor(), SampleCt, 2); // na razie wystarczy jeden set

    Sampler areaLightSampler = Sampler(new NRooks(0), new SquareDistributor(), SampleCt, 2);

    QTime time;

    switch(ui->comboBox_2->currentIndex())
    {
        case 0:
            /*
            scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, blueMat));
            scene->addObj(new Sphere(Vec3(4, 0, 0), 1.5, greenMetal));
            scene->addObj(new Sphere(Vec3(0, 0, 3), 2, redMir));
            scene->addObj(new Sphere(Vec3(-7, 0, 3), 2, grayMat));
            scene->addObj(new Sphere(Vec3(8, 0, 4), 2.5, redMetal));
            scene->addObj(new Sphere(Vec3(2, 0, 8), 1, blueMir));
            scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMat));
            scene->addObj(new Plane(Vec3(0, 0, 10), Vec3(0, 0, 1), whiteMat));
            */

            for(int i = 0; i < 8; i+=2)
            {
                for(int j = -5; j < 6; j+=2)
                {
                    scene->addObj(new Sphere(Vec3(j, i, 3), 0.9, orangeMat));
                }
            }
            scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMat));
            scene->addObj(new Plane(Vec3(0, 0, 7), Vec3(0, 0, 1), grayMir));
            scene->addLight(new Light(Vec3(1, 3, 1), white));

            tracer->raytrace(scene, camera, W, H, paper);

            ui->label->setPixmap(QPixmap::fromImage(*paper));
            break;
        case 1:
            scene->addObj(new Sphere(Vec3(0, 0, 0), 9, new Reflective(white, 0, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(-4, 0, 0), 2, new Reflective(white, 0, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(4, 0, 0), 2, new Reflective(white, 0, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(0, 0, 3), 2, new Reflective(white, 0, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), grayMir));

            scene->addLight(new Light(Vec3(0, 3, -3), red, areaLightSampler, 1));
            scene->addLight(new Light(Vec3(0, 3, 5), green, areaLightSampler, 1));
            scene->addLight(new Light(Vec3(3, 3, -3), blue, areaLightSampler, 1));
            scene->addLight(new Light(Vec3(-3, 3, 5), white, areaLightSampler, 1));

            tracer->raytrace(scene, camera, W, H, paper, antiAlias);

            ui->label->setPixmap(QPixmap::fromImage(*paper));
            break;

        case 2:
            scene->addObj(new Sphere(Vec3(-7, 2.5, 7), 2.5, new Reflective(white, 1, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(-7, 1.5, 3.5), 1.5, new Reflective(red, 1, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(0, 2.5, 7), 2.5, new Reflective(white, 0.75, 1, 300, 0.25)));
            scene->addObj(new Sphere(Vec3(0, 1.5, 3), 1.5, new Reflective(blue, 1, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(7, 2.5, 7), 2.5, new Reflective(white, 0.0, 1, 300, 1.0)));
            scene->addObj(new Sphere(Vec3(7, 1.5, 2.5), 1.5, new Reflective(green, 1, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(0, 0, 0), Vec3(0, 1, 0), new Reflective(white, 0.9, 1, 300, 0.1)));
            scene->addObj(new Plane(Vec3(0, 0, 13), Vec3(0, 0, -1), new Reflective(white, 1.0, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(0, 0, -14), Vec3(0, 0, 1), new Reflective(white, 1.0, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(11, 0, 0), Vec3(-1, 0, 0), new Reflective(white, 1.0, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(-11, 0, 0), Vec3(1, 0, 0), new Reflective(white, 1.0, 1, 300, 0.0)));

            scene->addLight(new Light(Vec3(0, 5, -2), white, areaLightSampler, 1));

            time.start(); // rozpoczęcie odliczania

            tracer->raytrace(scene, camera, W, H, paper, antiAlias);

            qDebug() << "Finly:"<< QTime::fromMSecsSinceStartOfDay(time.restart()); // wyświetlenie końcowego czasu

            paper->save("test3.png");
            ui->label->setPixmap(QPixmap::fromImage(*paper));
            break;

        case 3:
            scene->addObj(new Sphere(Vec3(0, 1, 2), 1.7, new Reflective(orange, 0.95, 1, 500, 0.05)));
            scene->addObj(new Plane(Vec3(0, 0, 0), Vec3(0, 1, 0), new Reflective(white, 1, 1, 300, 0.0)));

            scene->addLight(new Light(Vec3(3, 3, 2), white, areaLightSampler, 2.3));

            tracer->raytrace(scene, camera, W, H, paper, antiAlias);
;
            ui->label->setPixmap(QPixmap::fromImage(*paper));
            break;

        case 4:
            scene->addObj(new Sphere(Vec3(-4, 2, 2), 2, new Reflective(orange, 1, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(5, 2, 6), 2, new Reflective(white, 1, 1, 300, 0.0)));
            scene->addObj(new Sphere(Vec3(1, 2, 13), 2, new Reflective(yellow, 1, 1, 300, 0.0)));
            scene->addObj(new Plane(Vec3(0, 0, 0), Vec3(0, 1, 0), new Reflective(gray, 1.00, 1, 300, 0.0)));

            scene->addLight(new Light(Vec3(3, 10, 6), white, areaLightSampler, 4.0));

            time.start(); // rozpoczęcie odliczania

            tracer->raytrace(scene, camera, W, H, paper, antiAlias);

            qDebug() << "Finly:"<< QTime::fromMSecsSinceStartOfDay(time.restart()); // wyświetlenie końcowego czasu

            paper->save("test.png");
            ui->label->setPixmap(QPixmap::fromImage(*paper));
            break;
    }

}
