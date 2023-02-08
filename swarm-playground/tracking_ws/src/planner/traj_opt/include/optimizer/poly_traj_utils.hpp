#pragma once

#include "root_finder.hpp"

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace poly_traj
{

    // Polynomial order and trajectory dimension are fixed here
    typedef Eigen::Matrix<double, 3, 6> CoefficientMat;
    typedef Eigen::Matrix<double, 3, 5> VelCoefficientMat;
    typedef Eigen::Matrix<double, 3, 4> AccCoefficientMat;

    class Piece
    {
    private:
        double duration;
        CoefficientMat coeffMat;

    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat &cMat)
            : duration(dur), coeffMat(cMat) {}

        inline int getDim() const
        {
            return 3;
        }

        inline int getOrder() const
        {
            return 5;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline const CoefficientMat &getCoeffMat() const
        {
            return coeffMat;
        }

        inline VelCoefficientMat getVelCoeffMat() const
        {
            VelCoefficientMat velCoeffMat;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                velCoeffMat.col(i) = n * coeffMat.col(i);
                n++;
            }
            return velCoeffMat;
        }

        inline Eigen::Vector3d getPos(const double &t) const
        {
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            double tn = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Vector3d getVel(const double &t) const
        {
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            double tn = 1.0;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Vector3d getAcc(const double &t) const
        {
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = 3; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Vector3d getJer(const double &t) const
        {
            Eigen::Vector3d jer(0.0, 0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = 2; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline CoefficientMat normalizePosCoeffMat() const
        {
            CoefficientMat nPosCoeffsMat;
            double t = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nPosCoeffsMat;
        }

        inline VelCoefficientMat normalizeVelCoeffMat() const
        {
            VelCoefficientMat nVelCoeffMat;
            int n = 1;
            double t = duration;
            for (int i = 4; i >= 0; i--)
            {
                nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nVelCoeffMat;
        }

        inline AccCoefficientMat normalizeAccCoeffMat() const
        {
            AccCoefficientMat nAccCoeffMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            for (int i = 3; i >= 0; i--)
            {
                nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nAccCoeffMat;
        }

        inline double getMaxVelRate() const
        {
            Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getVel((*it) * duration).squaredNorm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        inline double getMaxAccRate() const
        {
            Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        // GaaiLam
        inline bool project_pt(const Eigen::Vector3d &pt,
                               double &tt, Eigen::Vector3d &pro_pt)
        {
            // 2*(p-p0)^T * \dot{p} = 0
            auto l_coeff = getCoeffMat();
            l_coeff.col(5) = l_coeff.col(5) - pt;
            auto r_coeff = getVelCoeffMat();
            Eigen::VectorXd eq = Eigen::VectorXd::Zero(2 * 5);
            for (int j = 0; j < l_coeff.rows(); ++j)
            {
                eq = eq + RootFinder::polyConv(l_coeff.row(j), r_coeff.row(j));
            }
            double l = -0.0625;
            double r = duration + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);
            // std::cout << "# roots: " << roots.size() << std::endl;
            double min_dist = -1;
            for (const auto &root : roots)
            {
                // std::cout << "root: " << root << std::endl;
                if (root < 0 || root > duration)
                {
                    continue;
                }
                if (getVel(root).norm() < 1e-6)
                { // velocity == 0, ignore it
                    continue;
                }
                // std::cout << "find min!" << std::endl;
                Eigen::Vector3d p = getPos(root);
                // std::cout << "p: " << p.transpose() << std::endl;
                double distance = (p - pt).norm();
                if (distance < min_dist || min_dist < 0)
                {
                    min_dist = distance;
                    tt = root;
                    pro_pt = p;
                }
            }
            return min_dist > 0;
        }

        inline bool intersection_plane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       double &tt, Eigen::Vector3d &pt) const
        {
            // (pt - p)^T * v = 0
            auto coeff = getCoeffMat();
            coeff.col(5) = coeff.col(5) - p;
            Eigen::VectorXd eq = coeff.transpose() * v;
            double l = -0.0625;
            double r = duration + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);
            for (const auto &root : roots)
            {
                tt = root;
                pt = getPos(root);
                return true;
            }
            return false;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class Trajectory
    {
    private:
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

    public:
        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                   const std::vector<CoefficientMat> &cMats)
        {
            int N = std::min(durs.size(), cMats.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i]);
            }
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::MatrixXd getPositions() const
        {
            int N = getPieceNum();
            Eigen::MatrixXd positions(3, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(5);
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
            return positions;
        }

        inline const Piece &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline Pieces::const_iterator begin() const
        {
            return pieces.begin();
        }

        inline Pieces::const_iterator end() const
        {
            return pieces.end();
        }

        inline Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                 const CoefficientMat &cMat)
        {
            pieces.emplace_back(dur, cMat);
            return;
        }

        inline void append(const Trajectory &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        inline Eigen::Vector3d getPos(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        inline Eigen::Vector3d getVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::Vector3d getAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::Vector3d getJer(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Vector3d getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(5);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector3d getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(4);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector3d getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline double getMaxVelRate() const
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
            }
            return feasible;
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }

        // GaaiLam
        inline Piece getPiece(int i) const
        {
            return pieces[i];
        }

        inline bool project_pt(const Eigen::Vector3d &pt,
                               int &ii, double &tt, Eigen::Vector3d &pro_pt)
        {
            bool find_project_pt = false;
            for (int i = 0; i < getPieceNum(); ++i)
            {
                auto piece = pieces[i];
                if (piece.project_pt(pt, tt, pro_pt))
                {
                    ii = i;
                    find_project_pt = true;
                    break;
                }
            }
            if (!find_project_pt)
            {
                // std::cout << "\033[32m" << "cannot project pt to traj" << "\033[0m" << std::endl;
                // std::cout << "pt: " << pt.transpose() << std::endl;
                // assert(false);
            }
            return find_project_pt;
        }
        inline bool intersection_plane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       int &ii, double &tt, Eigen::Vector3d &pt)
        {
            for (int i = 0; i < getPieceNum(); ++i)
            {
                const auto &piece = pieces[i];
                if (piece.intersection_plane(p, v, tt, pt))
                {
                    ii = i;
                    return true;
                }
            }
            return false;
        }

        inline std::vector<Eigen::Vector3d> way_points()
        {
            std::vector<Eigen::Vector3d> pts;
            for (int i = 0; i < getPieceNum(); ++i)
            {
                pts.push_back(pieces[i].getPos(0));
            }
            return pts;
        }

        // zxzx
        inline std::pair<int, double> locatePieceIdxWithRatio(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            std::pair<int, double> idx_ratio;
            idx_ratio.first = idx;
            idx_ratio.second = t / dur;
            return idx_ratio;
        }

        inline Eigen::Vector3d getPoswithIdxRatio(double t, std::pair<int, double> &idx_ratio) const
        {
            idx_ratio = locatePieceIdxWithRatio(t);
            return pieces[idx_ratio.first].getPos(t);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

        inline void operator=(const BandedSystem &bs)
        {
            ptrData = nullptr;
            create(bs.N, bs.lowerBw, bs.upperBw);
            memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solveAdj(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class MinJerkOpt
    {
    public:
        inline void operator=(const MinJerkOpt &mjo)
        {
            N = mjo.N;
            headPVA = mjo.headPVA;
            tailPVA = mjo.tailPVA;
            T1 = mjo.T1;
            A = mjo.A;
            b = mjo.b;
            T2 = mjo.T2;
            T3 = mjo.T3;
            T4 = mjo.T4;
            T5 = mjo.T5;
            gdC = mjo.gdC;
        }
        ~MinJerkOpt() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        Eigen::VectorXd T1;
        BandedSystem A;
        Eigen::MatrixXd b;

        // Temp variables
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
        Eigen::MatrixXd gdC;

    private:
        template <typename EIGENVEC>
        inline void addGradJbyT(EIGENVEC &gdT) const
        {
            for (int i = 0; i < N; i++)
            {
                gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                          288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                          576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                          2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                          3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }

        template <typename EIGENMAT>
        inline void addGradJbyC(EIGENMAT &gdC) const
        {
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                                      720.0 * b.row(6 * i + 4) * T4(i) +
                                      1440.0 * b.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                                      384.0 * b.row(6 * i + 4) * T3(i) +
                                      720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                                      144.0 * b.row(6 * i + 4) * T2(i) +
                                      240.0 * b.row(6 * i + 5) * T3(i);
            }
            return;
        }

        inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
        {
            A.solveAdj(gdC);
            return;
        }

        template <typename EIGENVEC>
        inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const
        {
            Eigen::MatrixXd B1(6, 3), B2(3, 3);

            Eigen::RowVector3d negVel, negAcc, negJer, negSnp, negCrk;

            for (int i = 0; i < N - 1; i++)
            {
                negVel = -(b.row(i * 6 + 1) +
                           2.0 * T1(i) * b.row(i * 6 + 2) +
                           3.0 * T2(i) * b.row(i * 6 + 3) +
                           4.0 * T3(i) * b.row(i * 6 + 4) +
                           5.0 * T4(i) * b.row(i * 6 + 5));
                negAcc = -(2.0 * b.row(i * 6 + 2) +
                           6.0 * T1(i) * b.row(i * 6 + 3) +
                           12.0 * T2(i) * b.row(i * 6 + 4) +
                           20.0 * T3(i) * b.row(i * 6 + 5));
                negJer = -(6.0 * b.row(i * 6 + 3) +
                           24.0 * T1(i) * b.row(i * 6 + 4) +
                           60.0 * T2(i) * b.row(i * 6 + 5));
                negSnp = -(24.0 * b.row(i * 6 + 4) +
                           120.0 * T1(i) * b.row(i * 6 + 5));
                negCrk = -120.0 * b.row(i * 6 + 5);

                B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;

                gdT(i) += B1.cwiseProduct(adjGdC.block<6, 3>(6 * i + 3, 0)).sum();
            }

            negVel = -(b.row(6 * N - 5) +
                       2.0 * T1(N - 1) * b.row(6 * N - 4) +
                       3.0 * T2(N - 1) * b.row(6 * N - 3) +
                       4.0 * T3(N - 1) * b.row(6 * N - 2) +
                       5.0 * T4(N - 1) * b.row(6 * N - 1));
            negAcc = -(2.0 * b.row(6 * N - 4) +
                       6.0 * T1(N - 1) * b.row(6 * N - 3) +
                       12.0 * T2(N - 1) * b.row(6 * N - 2) +
                       20.0 * T3(N - 1) * b.row(6 * N - 1));
            negJer = -(6.0 * b.row(6 * N - 3) +
                       24.0 * T1(N - 1) * b.row(6 * N - 2) +
                       60.0 * T2(N - 1) * b.row(6 * N - 1));

            B2 << negVel, negAcc, negJer;

            gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, 3>(6 * N - 3, 0)).sum();

            return;
        }

        template <typename EIGENMAT>
        inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
        {
            for (int i = 0; i < N - 1; i++)
            {
                // gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
                gdInP.col(i) = adjGdC.row(6 * i + 5).transpose(); // zxzx
            }
            return;
        }

        template <typename EIGENVEC>
        inline void addTimeIntPenalty(const Eigen::VectorXi cons,
                                      const Eigen::VectorXi &idxHs,
                                      const std::vector<Eigen::MatrixXd> &cfgHs,
                                      const double vmax,
                                      const double amax,
                                      const Eigen::Vector3d ci,
                                      double &cost,
                                      EIGENVEC &gdT,
                                      Eigen::MatrixXd &gdC) const
        {
            double pena = 0.0;
            const double vmaxSqr = vmax * vmax;
            const double amaxSqr = amax * amax;

            Eigen::Vector3d pos, vel, acc, jer;
            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
            Eigen::Vector3d outerNormal;
            int K;
            double violaPos, violaVel, violaAcc;
            double violaPosPenaD, violaVelPenaD, violaAccPenaD;
            double violaPosPena, violaVelPena, violaAccPena;
            Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaAc;
            double gradViolaVt, gradViolaAt;
            double omg;

            int innerLoop, idx;
            for (int i = 0; i < N; i++)
            {
                const auto &c = b.block<6, 3>(i * 6, 0);
                step = T1(i) / cons(i);
                s1 = 0.0;
                innerLoop = cons(i) + 1;
                for (int j = 0; j < innerLoop; j++)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                    alpha = 1.0 / cons(i) * j;
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    violaVel = vel.squaredNorm() - vmaxSqr;
                    violaAcc = acc.squaredNorm() - amaxSqr;

                    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

                    idx = idxHs(i);
                    K = cfgHs[idx].cols();
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = cfgHs[idx].col(k).head<3>();
                        violaPos = outerNormal.dot(pos - cfgHs[idx].col(k).tail<3>());
                        if (violaPos > 0.0)
                        {
                            violaPosPenaD = violaPos * violaPos;
                            violaPosPena = violaPosPenaD * violaPos;
                            violaPosPenaD *= 3.0;
                            gdC.block<6, 3>(i * 6, 0) += omg * step * ci(0) * violaPosPenaD * beta0 * outerNormal.transpose();
                            gdT(i) += omg * (ci(0) * violaPosPenaD * alpha * outerNormal.dot(vel) * step +
                                             ci(0) * violaPosPena / cons(i));
                            pena += omg * step * ci(0) * violaPosPena;
                        }
                    }

                    if (violaVel > 0.0)
                    {
                        violaVelPenaD = violaVel * violaVel;
                        violaVelPena = violaVelPenaD * violaVel;
                        violaVelPenaD *= 3.0;
                        gradViolaVc = 2.0 * beta1 * vel.transpose();
                        gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
                        gdC.block<6, 3>(i * 6, 0) += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
                        gdT(i) += omg * (ci(1) * violaVelPenaD * gradViolaVt * step +
                                         ci(1) * violaVelPena / cons(i));
                        pena += omg * step * ci(1) * violaVelPena;
                    }

                    if (violaAcc > 0.0)
                    {
                        violaAccPenaD = violaAcc * violaAcc;
                        violaAccPena = violaAccPenaD * violaAcc;
                        violaAccPenaD *= 3.0;
                        gradViolaAc = 2.0 * beta2 * acc.transpose();
                        gradViolaAt = 2.0 * alpha * acc.transpose() * jer;
                        gdC.block<6, 3>(i * 6, 0) += omg * step * ci(2) * violaAccPenaD * gradViolaAc;
                        gdT(i) += omg * (ci(2) * violaAccPenaD * gradViolaAt * step +
                                         ci(2) * violaAccPena / cons(i));
                        pena += omg * step * ci(2) * violaAccPena;
                    }

                    s1 += step;
                }
            }

            cost += pena;
            return;
        }

    public:
        inline void reset(const Eigen::Matrix3d &headState,
                          const Eigen::Matrix3d &tailState,
                          const int &pieceNum)
        {
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            T1.resize(N);
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 3);
            gdC.resize(6 * N, 3);
            // gdT.resize(6 * N);
            return;
        }

        inline void generate(const Eigen::MatrixXd &inPs,
                             const Eigen::VectorXd &ts)
        {
            if (inPs.cols() == 0)
            {

                T1(0) = ts(0);
                double t1_inv = 1.0 / T1(0);
                double t2_inv = t1_inv * t1_inv;
                double t3_inv = t2_inv * t1_inv;
                double t4_inv = t2_inv * t2_inv;
                double t5_inv = t4_inv * t1_inv;
                CoefficientMat coeffMatReversed;
                coeffMatReversed.col(5) = 0.5 * (tailPVA.col(2) - headPVA.col(2)) * t3_inv -
                                          3.0 * (headPVA.col(1) + tailPVA.col(1)) * t4_inv +
                                          6.0 * (tailPVA.col(0) - headPVA.col(0)) * t5_inv;
                coeffMatReversed.col(4) = (-tailPVA.col(2) + 1.5 * headPVA.col(2)) * t2_inv +
                                          (8.0 * headPVA.col(1) + 7.0 * tailPVA.col(1)) * t3_inv +
                                          15.0 * (-tailPVA.col(0) + headPVA.col(0)) * t4_inv;
                coeffMatReversed.col(3) = (0.5 * tailPVA.col(2) - 1.5 * headPVA.col(2)) * t1_inv -
                                          (6.0 * headPVA.col(1) + 4.0 * tailPVA.col(1)) * t2_inv +
                                          10.0 * (tailPVA.col(0) - headPVA.col(0)) * t3_inv;
                coeffMatReversed.col(2) = 0.5 * headPVA.col(2);
                coeffMatReversed.col(1) = headPVA.col(1);
                coeffMatReversed.col(0) = headPVA.col(0);
                b = coeffMatReversed.transpose();
            }
            else
            {
                T1 = ts;
                T2 = T1.cwiseProduct(T1);
                T3 = T2.cwiseProduct(T1);
                T4 = T2.cwiseProduct(T2);
                T5 = T4.cwiseProduct(T1);

                A.reset();
                b.setZero();

                A(0, 0) = 1.0;
                A(1, 1) = 1.0;
                A(2, 2) = 2.0;
                b.row(0) = headPVA.col(0).transpose();
                b.row(1) = headPVA.col(1).transpose();
                b.row(2) = headPVA.col(2).transpose();

                for (int i = 0; i < N - 1; i++)
                {
                    A(6 * i + 3, 6 * i + 3) = 6.0;
                    A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                    A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                    A(6 * i + 3, 6 * i + 9) = -6.0;
                    A(6 * i + 4, 6 * i + 4) = 24.0;
                    A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                    A(6 * i + 4, 6 * i + 10) = -24.0;
                    A(6 * i + 5, 6 * i) = 1.0;
                    A(6 * i + 5, 6 * i + 1) = T1(i);
                    A(6 * i + 5, 6 * i + 2) = T2(i);
                    A(6 * i + 5, 6 * i + 3) = T3(i);
                    A(6 * i + 5, 6 * i + 4) = T4(i);
                    A(6 * i + 5, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i) = 1.0;
                    A(6 * i + 6, 6 * i + 1) = T1(i);
                    A(6 * i + 6, 6 * i + 2) = T2(i);
                    A(6 * i + 6, 6 * i + 3) = T3(i);
                    A(6 * i + 6, 6 * i + 4) = T4(i);
                    A(6 * i + 6, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i + 6) = -1.0;
                    A(6 * i + 7, 6 * i + 1) = 1.0;
                    A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                    A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                    A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                    A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                    A(6 * i + 7, 6 * i + 7) = -1.0;
                    A(6 * i + 8, 6 * i + 2) = 2.0;
                    A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                    A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                    A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                    A(6 * i + 8, 6 * i + 8) = -2.0;

                    b.row(6 * i + 5) = inPs.col(i).transpose();
                }

                A(6 * N - 3, 6 * N - 6) = 1.0;
                A(6 * N - 3, 6 * N - 5) = T1(N - 1);
                A(6 * N - 3, 6 * N - 4) = T2(N - 1);
                A(6 * N - 3, 6 * N - 3) = T3(N - 1);
                A(6 * N - 3, 6 * N - 2) = T4(N - 1);
                A(6 * N - 3, 6 * N - 1) = T5(N - 1);
                A(6 * N - 2, 6 * N - 5) = 1.0;
                A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
                A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
                A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
                A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
                A(6 * N - 1, 6 * N - 4) = 2;
                A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
                A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
                A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

                b.row(6 * N - 3) = tailPVA.col(0).transpose();
                b.row(6 * N - 2) = tailPVA.col(1).transpose();
                b.row(6 * N - 1) = tailPVA.col(2).transpose();

                A.factorizeLU();
                A.solve(b);

                return;
            }
        }

        inline const Eigen::MatrixXd &get_b() const
        {
            return b;
        }

        inline const Eigen::VectorXd &get_T1() const
        {
            return T1;
        }

        inline Eigen::MatrixXd &get_gdC()
        {
            return gdC;
        }

        // inline Eigen::MatrixXd get_gdT() const
        // {
        //     return gdT;
        // }

        // inline Eigen::MatrixXd get_gdT(size_t i) const
        // {
        //     return gdT(i);
        // }

        inline double getTrajJerkCost() const
        {
            double objective = 0.0;
            for (int i = 0; i < N; i++)
            {
                objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                             144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                             192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                             240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                             720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                             720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return objective;
        }

        inline Trajectory getTraj(void) const
        {
            Trajectory traj;
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

        inline Eigen::MatrixXd getInitConstraintPoints(const int K) const
        {
            Eigen::MatrixXd pts(3, N * K + 1);
            Eigen::Vector3d pos;
            Eigen::Matrix<double, 6, 1> beta0;
            double s1, s2, s3, s4, s5;
            double step;
            int i_dp = 0;

            for (int i = 0; i < N; ++i)
            {
                const auto &c = b.block<6, 3>(i * 6, 0);
                step = T1(i) / K;
                s1 = 0.0;
                double t = 0;
                // innerLoop = K;

                for (int j = 0; j <= K; ++j)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    pos = c.transpose() * beta0;
                    pts.col(i_dp) = pos;

                    s1 += step;
                    if (j != K || (j == K && i == N - 1))
                    {
                        ++i_dp;
                    }
                }
            }

            return pts;
        }

        template <typename EIGENVEC, typename EIGENMAT>
        inline void getGrad2TP(EIGENVEC &gdT,
                               EIGENMAT &gdInPs)
        {
            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs);
        }

        template <typename EIGENVEC>
        inline void initGradCost(EIGENVEC &gdT,
                                 double &cost)
        {
            // printf( "gdInPs=%d\n", gdInPs.size() );

            gdT.setZero();
            gdC.setZero();
            cost = getTrajJerkCost();
            addGradJbyT(gdT);
            addGradJbyC(gdC);
        }

        template <typename EIGENVEC, typename EIGENMAT>
        inline void evalTrajCostGrad(const Eigen::VectorXi &cons,
                                     const Eigen::VectorXi &idxHs,
                                     const std::vector<Eigen::MatrixXd> &cfgHs,
                                     const double &vmax,
                                     const double &amax,
                                     const Eigen::Vector3d &ci,
                                     double &cost,
                                     EIGENVEC &gdT,
                                     EIGENMAT &gdInPs)
        {
            gdT.setZero();
            gdInPs.setZero();
            gdC.setZero();

            cost = getTrajJerkCost();
            addGradJbyT(gdT);
            addGradJbyC(gdC);

            addTimeIntPenalty(cons, idxHs, cfgHs, vmax, amax, ci, cost, gdT, gdC);

            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} //namespace poly_traj
