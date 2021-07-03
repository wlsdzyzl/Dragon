#include "BasicGeometry.h"
namespace dragon
{
namespace geometry
{
    // Matrix4 Se3ToSE3(const Vector6 &input)
    // {
    //     auto tmp = Sophus::SE3Group<scalar>::exp(input);
    //     return tmp.matrix();
    // }
    // Vector6 SE3ToSe3(const Matrix4 &input)
    // {
    //     Sophus::SE3Group<scalar>  tmp(input);
    //     return tmp.log(); 
    // }
    Point3 TransformPoint(const Matrix4 &T, const Point3 &point)
    {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            return new_point.head<3>() / new_point(3);        
    }
    void TransformPoints(const Matrix4 &T, Point3List &points)
    {
        for (auto& point : points) 
        {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            point = new_point.head<3>() / new_point(3);
        }
    }
    void TransformNormals(const Matrix4 &T, Point3List &normals)
    {
        for (auto& normal : normals) 
        {
            Vector4 new_normal =
                T *Vector4(normal(0), normal(1), normal(2), 0.0);
            normal = new_normal.head<3>();
        }
    }  
    double ComputeTriangleArea(const PointX &a, const PointX &b, const PointX &c)
    {
        auto edge1 = b-a;
        auto edge2 = c-a;

        // std::cout<<"area: "<<area<<" "<< <<std::endl;

        return 0.5 * edge1.norm() * edge2.norm() * Sin(edge1, edge2);
    }
    std::tuple<Point3, double> CircumCenter(const Point3 &a, const Point3 &b, const Point3 &c)
    {
        Vector3 ac = c - a ;
        Vector3 ab = b - a ;
        Vector3 abXac = ab.cross( ac ) ;
        // this is the vector from a TO the circumsphere center
        Point3 o = (abXac.cross( ab )*ac.squaredNorm() + ac.cross( abXac )*ab.squaredNorm()) / (2.f*abXac.squaredNorm()) ;
        double r = o.norm() ;
        // The 3 space coords of the circumsphere center then:
        return std::make_tuple( a  +  o, r) ;
    }
    std::tuple<Point2, double, double> CircumCenter(const Point2 &a, const Point2 &b, const Point2 &c)
    {
        Point2 circumcenter;
        double xba, yba, xca, yca;
        double balength, calength;
        double denominator;
        double xcirca, ycirca;
        /* Use coordinates relative to point `a' of the triangle. */
        xba = b[0] - a[0];
        yba = b[1] - a[1];
        xca = c[0] - a[0];
        yca = c[1] - a[1];
        /* Squares of lengths of the edges incident to `a'. */
        balength = xba * xba + yba * yba;
        calength = xca * xca + yca * yca;
        /* Calculate the denominator of the formulae. */
        double d = xba * yca - yba * xca;
        denominator = 0.5 / d;


        xcirca = (yca * balength - yba * calength) * denominator;
        ycirca = (xba * calength - xca * balength) * denominator;
        circumcenter[0] = xcirca;
        circumcenter[1] = ycirca;
        double r = circumcenter.norm();
        return std::make_tuple(a + circumcenter, r, d);

    }
    int TriangleType(const PointX &a, const PointX &b, const PointX &c)
    {
        VectorX ab = b - a;
        VectorX ac = c - a;
        VectorX ba = -ab;
        VectorX bc = c - b;
        VectorX ca = -ac;
        VectorX cb = -bc;
        double a1 = ac.dot(ab);
        double a2 = ba.dot(bc);
        double a3 = ca.dot(cb);
        // right angle
        if(std::fabs(a1) <DRAGON_EPS || std::fabs(a2) < DRAGON_EPS || std::fabs(a3) < DRAGON_EPS)
        return 1;
        // obtuse angle
        if(a1 < 0 || a2 < 0 || a3 < 0)
        return 2;
        // acute angle
        return 0;
    }
    int AngleType(const VectorX &a, const VectorX &b)
    {
        double dot = a.dot(b);
        if(std::fabs(dot) < DRAGON_EPS)
        return 1;
        if(dot < 0)
        return 2;
        return 0;
    }  
    double AngleOfVector(const VectorX &a, const VectorX &b)
    {
        return std::acos(a.dot(b) / (a.norm() * b.norm()));
    }
    void AddToCoefficientTriplet(std::vector<Eigen::Triplet<scalar>> &coefficients,
        int start_row, int start_col, const MatrixX &JTJ)
    {
        int rows = JTJ.rows();
        int cols = JTJ.cols();
        for(int i = 0; i != rows; ++i)
        {
            for(int j = 0; j != cols; ++j)
            {
                if(JTJ(i, j) != 0)
                    coefficients.push_back(Eigen::Triplet<scalar>(i + start_row, j + start_col, JTJ(i,j)));
            }
        }
    }
    Matrix3 RotationMatrix(const Vector3 &_axis, double angle)
    {
        //Rodrigues
        // Matrix3 rotation_matrix;
        Vector3 axis = _axis.normalized();
        // Matrix3 aat =  axis * axis.transpose();
        // Matrix3 A = GetSkewSymmetricMatrix(axis);
        // rotation_matrix = aat + (Matrix3::Identity()-aat )*cos(angle) + sin(angle) * A;
        return Eigen::AngleAxis<geometry::scalar>(angle, axis).toRotationMatrix();
    }
    Matrix3 GetSkewSymmetricMatrix(const Vector3 &t)
    {
        Matrix3 t_hat;
        t_hat << 0, -t(2), t(1),
                t(2), 0, -t(0),
                -t(1), t(0), 0;
        return t_hat;        
    }
    geometry::Matrix4 RandomTransformation()
    {
        std::random_device rd;
        std::default_random_engine e(rd());
        std::uniform_real_distribution<double> u(0, 1.0);
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(u(e) * M_PI, ::Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(u(e) * M_PI, ::Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(u(e) * M_PI, ::Eigen::Vector3d::UnitX());
        Matrix4 t = Matrix4::Identity();
        t.block<3, 3>(0, 0) = R.cast<scalar>();
        t(0, 3) = u(e) * 5;
        t(1, 3) = u(e) * 5;
        t(2, 3) = u(e) * 5;
        return t;
    }
    std::tuple<Point3, double , double> FitPlane(const Point3List & _points)
    {
        if(_points.size() < 3)
        {
            std::cout<<YELLOW<< "[FitPlane]::[WARNING]::The Number of points is less than 3."<<RESET << std::endl;
            return std::make_tuple(Point3(0,0,0),0,0);
        }
        Point3 mean_point;
        Point3 sum_point;
        sum_point.setZero();
        for(size_t i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Matrix3 W, W_tmp;
        W.setZero();
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Point3 normal = U.block<3,1>(0,2); 
        //std::cout<<U<<std::endl;   
        normal.normalize();    

/*
        Point3 temp = (_points[0] - mean_point);
        temp.normalize();
        if(temp.dot(normal) >= 0 )
        normal = -normal;
*/
        double d = - mean_point.transpose() * normal;
        
        /*
        double residual = 0;
        
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }*/
        double indicator = singular_values(2) / singular_values(1);
        return std::make_tuple(normal,d, indicator);
    }

    std::tuple<Vector2, double, double > FitLine(const Point2List & _points)
    {
        if(_points.size() < 2)
        {
            std::cout<<YELLOW<< "[FitLine]::[WARNING]::The Number of points is less than 2."<<RESET << std::endl;
            return std::make_tuple(Vector2(0,0),0,0);
        }
        Vector2 mean_point;
        Vector2 sum_point;
        sum_point.setZero();
        for(size_t i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Matrix2 W, W_tmp;
        W.setZero();
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Vector2 normal = U.block<2,1>(0,1); 
        //std::cout<<singular_values(0) << " "<<singular_values(1)<<std::endl;   
        normal.normalize();    
        double d = - mean_point.transpose() * normal;
        /*
        double residual = 0;
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }
        residual /= _points.size();
        */
        //indicator of the curve, noise, and residual.
        double indicator = singular_values(1)/singular_values(0);
        //std::cout<<residual<<" "<<indicator<<std::endl;
        return std::make_tuple(normal,d, indicator);
    }
    double Cross3(const Point2 &a, const Point2 &b, const Point2 &c)
    { 
        return (c(0) - b(0)) * (a(1) - b(1)) - (c(1) - b(1)) * (a(0) - b(0));
    }
    bool InSegBoundingX(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)))
            return 1;
        return 0;        
    }
    bool InSegBoundingY(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;        
    }
    bool InSegBounding(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)) 
            && std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;
    }
    bool IsIntersecting(const Line & l1, const LineSegment &l2)
    {
        double min_x = std::min(l2.p0(0), l2.p1(0));
        double max_x = std::max(l2.p0(0), l2.p1(0));
        if(l1.n(1) == 0)
        {
            double x = -l1.d /l1.n(0);
            return min_x <= x && max_x >= x;
        }
        else if(l1.n(0) == 0)
        {
            double min_y = std::min(l2.p0(1), l2.p1(1));
            double max_y = std::max(l2.p0(1), l2.p1(1));
            double y = -l1.d /l1.n(1);
            return min_y <= y && max_y >= y;            
        }
        double y1 = (-l1.n(0) * min_x - l1.d) / l1.n(1);
        double y2 = (-l1.n(0) * max_x - l1.d) / l1.n(1);
        
        LineSegment l(Point2(min_x,y1), Point2(max_x, y2));

        return IsIntersecting(l,l2);
    }
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2)
    {
        double d1 = Cross3(l1.p0, l1.p1, l2.p0);
        double d2 = Cross3(l1.p0, l1.p1, l2.p1);
        double d3 = Cross3(l2.p0, l2.p1, l1.p0);
        double d4 = Cross3(l2.p0, l2.p1, l1.p1);
        
        if (((d1 < -DRAGON_EPS && d2 > DRAGON_EPS) || (d1 > DRAGON_EPS && d2 < -DRAGON_EPS)) && ((d3 < -DRAGON_EPS && d4 > DRAGON_EPS) || (d3 > DRAGON_EPS && d4 < -DRAGON_EPS)))
            return 1;
        if ((fabs(d1) <= DRAGON_EPS && InSegBounding(l1, l2.p0)) || (fabs(d2) <= DRAGON_EPS && InSegBounding(l1, l2.p1)) 
            || (fabs(d3) <= DRAGON_EPS && InSegBounding(l2, l1.p0)) || (fabs(d4) <= DRAGON_EPS && InSegBounding(l2, l1.p1)))
            return 1;
        return 0;
    }

    Line LineFromSeg(const LineSegment &s)
    {
        Line line;
        line.n = geometry::Vector2(s.p0(1) - s.p1(1), s.p1(0) - s.p0(0));
        line.n.normalize();
        line.d = - line.n.transpose() * s.p0;
        return line;
    }
    Point2 LineIntersect(const Line &a, const Line &b)
    {
        double x = a.n(1) * b.d - b.n(1) * a.d;
        double y = b.n(0) * a.d - a.n(0) * b.d;
        return Point2(x,y) / (a.n(0) * b.n(1) - b.n(0) * a.n(1));
    }
    Point2 SegIntersect(const LineSegment &s1, const LineSegment &s2)
    {
        Line l1 = LineFromSeg(s1);
        Line l2 = LineFromSeg(s2);
        return LineIntersect(l1, l2);
    }
    Point2 LineSegIntersect(const Line &a, const LineSegment & b)
    {
        Line l = LineFromSeg(b);
        return LineIntersect(a,l);
    }
    double Distance(const Point2 &a, const Point2 &b)
    {
        return (a - b).norm();
    }
    Point2 ProjectionPointToLine(const Line &a, const Point2 &p)
    {
        double tmp = a.n.transpose() * a.n;
        return  Point2( (a.n(1) * a.n(1) * p(0) - a.n(0) * a.n(1) * p(1) - a.n(0)* a.d)/ tmp,
         (a.n(0) * a.n(0) * p(1) -a.n(0) * a.n(1) * p(0) - a.n(1) * a.d) / tmp ) ;
    }
    Point2 ProjectionPointToLineSegment(const LineSegment &a, const Point2 &p)
    {
        return ProjectionPointToLine(LineFromSeg(a), p);
    }

    int CheckPointToLine(const Line &line, const Point2 &point)
    {
        double distance = line.n.transpose() * point + line.d;
        if(std::fabs(distance) < DRAGON_EPS) return 0;
        if(distance > 0) return 1;
        return -1;
        //if(distance == 0) return 0;
        //return distance; 
    }
    int CheckPointToLine(const Point2 &a, const Point2 &b, const Point2 &z )
    {
        // equal to CheckPointToLine(LineFromSeg(LineSegment(a,b)), z);
        double d =  a(0)* b(1) + a(1) * z(0) + b(0) * z(1) - z(0)* b(1) - a(1)*b(0) - a(0) * z(1);
        //std::cout<<d<<std::endl;
        // if(std::fabs(d) < DRAGON_EPS)
        // {
        //     std::cout<<YELLOW<<"Special case: Point is On Line."<<RESET<<std::endl;
        //     exit(0);
        // }
        if(std::fabs(d) < DRAGON_EPS) return 0;
        if(d > 0) return 1;
        return -1;
    }
    int CheckPointInTriangle(const Point2 & a, const Point2 &b, const Point2 & c, const Point2 &p)
    {
        // Check if a point is in the triangle. 
        // And we consider "the point is on the triangle" as "the point is in the triangle"
        int a_r = CheckPointToLine(a, b, p);
        int b_r = CheckPointToLine(b, c, p);
        int c_r = CheckPointToLine(c, a, p);
        //std::cout<<a_r<<" "<<b_r<<" "<<c_r<<std::endl;
        if(a_r == b_r && b_r == c_r)
        return 1;
        if((a_r == b_r && c_r == 0) || (a_r == c_r && b_r == 0) || (c_r == b_r && a_r == 0))
        return 1;
        if((a_r == b_r && a_r == 0) || (a_r == c_r && a_r == 0) || (c_r == b_r && c_r == 0))
        return 1;

        return 0;
    }
    // this code is from https://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
    // an elegant solution.
    int CheckPointProjectionInTriangle(const Point3& query_point,
                        const Point3& triangle_vertex_0,
                        const Point3& triangle_vertex_1,
                        const Point3& triangle_vertex_2, Point3 &projected_p)
    {
        // u=P2−P1
        Point3 u = triangle_vertex_1 - triangle_vertex_0;
        // v=P3−P1
        Point3 v = triangle_vertex_2 - triangle_vertex_0;
        // n=u×v
        Point3 n = u.cross(v);
        // w=P−P1
        Point3 w = query_point - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        float beta = w.cross(v).dot(n) / n.dot(n);
        float alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        projected_p = alpha * triangle_vertex_0 + beta * triangle_vertex_1 + gamma * triangle_vertex_2;
        return ((0 <= alpha) && (alpha <= 1) &&
                (0 <= beta)  && (beta  <= 1) &&
                (0 <= gamma) && (gamma <= 1));
    }
    //
    int CheckPointProjectionOnLineSegment(const Point3 &query_point, const Point3 &start, const Point3 &end,
                        Point3 &projected_p)
    {
        Point3 u = end - start;
        Point3 v = query_point - start;
        float beta = u.dot(v) / u.squaredNorm();
        float alpha = 1 - beta;
        projected_p = alpha * start + beta * end;
        return ((0 <= alpha) && (alpha <= 1) &&
                (0 <= beta) && (beta <= 1));
    }
    double ComputeAreaTriangle(Point2 a, Point2 b, Point2 c)
    {
        return std::fabs(0.5*(a(0) * b(1) + b(0) * c(1) + c(0) * a(1) - 
            a(0) * c(1) - b(0) * a(1) - c(0) * b(1) ));
    }
    double ComputeAreaConvexPoly(const Point2List &points)
    {
        int seed = 0; 
        int start = 1;
        int end = points.size();
        double area = 0;
        if(points.size() < 3) return 0;

        for(int i = start; i < end-1; ++i)
        {
            area += ComputeAreaTriangle(points[seed], points[i], points[i + 1]);
        }
        return area;
    }
    int CheckPointInConvexPoly(const geometry::Point2List &points, const Point2 &p )
    {
        // Same as the in-triangle-test
        int seed = 0;
        int start = 1;
        int end = points.size(); 
        int check = (start + end )/2;        
        if(end < 3) 
        {
            std::cout<<YELLOW<<"[CheckPointInConvexPoly]::[Warning]::Not a convex polygon."<<RESET<<std::endl;
            return -1;
        }

        while(true)
        {
            if(end - start == 2)
            {
                return CheckPointInTriangle(points[seed], points[start], points[end-1], p);
            }
            if(end <= start)
            {
                std::cout<<RED<<"[CheckPointInConvexPoly]::[ERROR]::Something wrong."<<RESET<<std::endl;
                //std::cout<<end<<" "<<start<<std::endl;
                return 0;                
            }

            int d = CheckPointToLine(points[seed], points[check], p);
            //CheckPointToLine(points[check],points[seed],p);
            if(d == 1)
            {
                start = check;
            }
            else
            {
                end = check + 1;
            }
            check = (start + end )/2;
        }
        return 0;
    }
    Line VerticalBisector(const Point2 & a, const Point2 &b)
    {
        Point2 mid = (a + b) / 2;
        Point2 n = (b-a).normalized();
        double d = -n.dot(mid);
        return Line(n, d); 
    }
    // choose svd if b is a vector
    geometry::VectorX SolveBySVD(const geometry::MatrixX &A, const geometry::VectorX &b)
    {
        geometry::VectorX res(A.cols());
        if(A.rows() != b.rows())
        {
            std::cout<<RED<<"[ERROR]::[Solver]::A and b must have the same rows."<<RESET<<std::endl;
            res.setZero();
            return res;
        }
        if(A.rows() >= A.cols())
        {
            //over determined
            res = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        }
        else
        {
            //under determined
            int start = A.cols() - A.rows();
            geometry::MatrixX newA = A.block(0, start, A.rows(), A.rows());
            geometry::VectorX new_res = newA.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
            res.setZero();
            res.block(start, 0, A.rows(), 1) = new_res; 
        }
        return res;
    }
    // choose lu if b is a matrix
    geometry::MatrixX SolveByLu(const geometry::MatrixX &A, const geometry::MatrixX &b)
    {
        //lu can be used to solve matrix as long as A and b has the same rows
        geometry::MatrixX res(A.cols(), b.cols());
        if(A.rows() != b.rows())
        {
            std::cout<<RED<<"[ERROR]::[Solver]::A and b must have the same rows."<<RESET<<std::endl;
            res.setZero();
            return res;
        }
        res = A.fullPivLu().solve(b);
        return res;
    }

    // for special cases, so it's not recommended
    geometry::VectorX SolveByThomas(const geometry::MatrixX &A, const geometry::VectorX &f)
    {
        int n = A.rows();
        geometry::MatrixX P(n, n), Q(n, n);
        P.setIdentity();
        Q.setIdentity();
        double last_q = 0;
        for(int i = 0; i != n; ++i)
        {
            double p, q;
            if(i == 0)
                p = A(i, i);
            else
            {
                p = A(i, i) - A(i, i-1) * last_q;
                P(i, i-1) = A(i, i-1);
            }
            P(i, i) = p;
            if(i != n-1)
            {
                q = A(i, i+1) / p;
                last_q = q;
                Q(i, i+1)= q;
            }
        }
        geometry::VectorX y(P.cols());
        // solve Py = f
        double last_y = 0;
        for(int i = 0; i != n; ++i)
        {
            double tmp_y;
            if(i == 0) tmp_y = f(0) / P(0, 0);
            else tmp_y = (f(i) - A(i, i-1) * last_y) / P(i, i);
            y(i) = tmp_y;
            last_y = tmp_y;
        }
        // solve Qx = y
        geometry::VectorX x(Q.cols());
        double last_x = 0;
        for(int i = 0; i != n; ++i)
        {
            int real_id = n-1-i;
            double tmp_x;
            if(i == 0) tmp_x = y(real_id);
            else tmp_x = (y(real_id) - Q(real_id, real_id+1) * last_x);
            x(real_id) = tmp_x;
            last_x = tmp_x;
        }        
        //x.reverseInPlace();
        return x;
    }
}
}