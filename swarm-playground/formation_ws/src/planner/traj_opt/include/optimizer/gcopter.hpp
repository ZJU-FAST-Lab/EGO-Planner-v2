#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "root_finder.hpp"
#include "lbfgs.hpp"
#include "geoutils.hpp"

#include <Eigen/Eigen>

#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>

typedef Eigen::Matrix<double, 3, 4> CoefficientMat;
typedef Eigen::Matrix<double, 3, 3> VelCoefficientMat;
typedef Eigen::Matrix<double, 3, 2> AccCoefficientMat;

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
        return 3;
    }

    inline double getDuration() const
    {
        return duration;
    }

    inline const CoefficientMat &getCoeffMat() const
    {
        return coeffMat;
    }

    inline Eigen::Vector3d getPos(const double &t) const
    {
        return coeffMat.col(3) +
               t * (coeffMat.col(2) +
                    t * (coeffMat.col(1) +
                         t * coeffMat.col(0)));
    }

    inline Eigen::Vector3d getVel(const double &t) const
    {
        return coeffMat.col(2) +
               (2.0 * t) * (coeffMat.col(1) +
                            (1.5 * t) * coeffMat.col(0));
    }

    inline Eigen::Vector3d getAcc(const double &t) const
    {
        return 2.0 * (coeffMat.col(1) +
                      (3.0 * t) * coeffMat.col(0));
    }

    inline Eigen::Vector3d getJer(const double &t) const
    {
        return 6.0 * coeffMat.col(0);
    }

    inline double getMaxVelRate() const
    {
        VelCoefficientMat nVelCoeffMat;
        nVelCoeffMat.col(2) = duration * coeffMat.col(2);
        nVelCoeffMat.col(1) = (2.0 * duration * duration) * coeffMat.col(1);
        nVelCoeffMat.col(0) = (3.0 * duration * duration * duration) * coeffMat.col(0);

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
            return getVel(0.0).norm();
        }
        else
        {
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), 0.0, 1.0,
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
        return std::sqrt(std::max(getAcc(0.0).squaredNorm(), getAcc(duration).squaredNorm()));
    }

    inline double getMaxJerRate() const
    {
        return getJer(0.0).norm();
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
            VelCoefficientMat nVelCoeffMat;
            nVelCoeffMat.col(2) = duration * coeffMat.col(2);
            nVelCoeffMat.col(1) = (2.0 * duration * duration) * coeffMat.col(1);
            nVelCoeffMat.col(0) = (3.0 * duration * duration * duration) * coeffMat.col(0);
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
        return getMaxAccRate() <= maxAccRate;
    }
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

private:
    int N;
    int lowerBw;
    int upperBw;
    // Compulsory nullptr initialization here
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
};

class MinAccOpt
{
public:
    MinAccOpt() = default;
    ~MinAccOpt() { A.destroy(); }

private:
    int N;
    Eigen::Matrix<double, 3, 2> headPV;
    Eigen::Matrix<double, 3, 2> tailPV;
    Eigen::VectorXd T1;
    BandedSystem A;
    Eigen::MatrixXd b;

    // Temp variables
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::MatrixXd gdC;

private:
    template <typename EIGENVEC>
    inline void addGradJbyT(EIGENVEC &gdT) const
    {
        for (int i = 0; i < N; i++)
        {
            gdT(i) += 4.0 * b.row(4 * i + 2).squaredNorm() +
                      24.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T1(i) +
                      36.0 * b.row(4 * i + 3).squaredNorm() * T2(i);
        }
        return;
    }

    template <typename EIGENMAT>
    inline void addGradJbyC(EIGENMAT &gdC) const
    {
        for (int i = 0; i < N; i++)
        {
            gdC.row(4 * i + 2) += 8.0 * b.row(4 * i + 2) * T1(i) +
                                  12.0 * b.row(4 * i + 3) * T2(i);

            gdC.row(4 * i + 3) += 12.0 * b.row(4 * i + 2) * T2(i) +
                                  24.0 * b.row(4 * i + 3) * T3(i);
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

        Eigen::Matrix<double, 4, 3> B1;
        Eigen::Matrix<double, 2, 3> B2;

        for (int i = 0; i < N - 1; i++)
        {
            // negative jerk
            B1.row(0) = -6.0 * b.row(i * 4 + 3);

            // negative velocity
            B1.row(1) = -(b.row(i * 4 + 1) +
                          2.0 * T1(i) * b.row(i * 4 + 2) +
                          3.0 * T2(i) * b.row(i * 4 + 3));
            B1.row(2) = B1.row(1);

            // negative acceleration
            B1.row(3) = -(2.0 * b.row(i * 4 + 2) +
                          6.0 * T1(i) * b.row(i * 4 + 3));

            gdT(i) += B1.cwiseProduct(adjGdC.block<4, 3>(4 * i + 2, 0)).sum();
        }

        // negative velocity
        B2.row(0) = -(b.row(4 * N - 3) +
                      2.0 * T1(N - 1) * b.row(4 * N - 2) +
                      3.0 * T2(N - 1) * b.row(4 * N - 1));

        // negative acceleration
        B2.row(1) = -(2.0 * b.row(4 * N - 2) +
                      6.0 * T1(N - 1) * b.row(4 * N - 1));

        gdT(N - 1) += B2.cwiseProduct(adjGdC.block<2, 3>(4 * N - 2, 0)).sum();

        return;
    }

    template <typename EIGENMAT>
    inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
    {
        for (int i = 0; i < N - 1; i++)
        {
            gdInP.col(i) += adjGdC.row(4 * i + 3).transpose();
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
        double s1, s2, s3;
        Eigen::Matrix<double, 4, 1> beta0, beta1, beta2, beta3;
        Eigen::Vector3d outerNormal;
        int K;
        double violaPos, violaVel, violaAcc;
        double violaPosPenaD, violaVelPenaD, violaAccPenaD;
        double violaPosPena, violaVelPena, violaAccPena;
        Eigen::Matrix<double, 4, 3> gradViolaVc, gradViolaAc;
        double gradViolaVt, gradViolaAt;
        double omg;

        int innerLoop, idx;
        for (int i = 0; i < N; i++)
        {
            const auto &c = b.block<4, 3>(i * 4, 0);
            step = T1(i) / cons(i);
            s1 = 0.0;
            innerLoop = cons(i) + 1;
            for (int j = 0; j < innerLoop; j++)
            {
                s2 = s1 * s1;
                s3 = s2 * s1;
                beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3;
                beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2;
                beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1;
                beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0;
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
                        gdC.block<4, 3>(i * 4, 0) += omg * step * ci(0) * violaPosPenaD * beta0 * outerNormal.transpose();
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
                    gdC.block<4, 3>(i * 4, 0) += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
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
                    gdC.block<4, 3>(i * 4, 0) += omg * step * ci(2) * violaAccPenaD * gradViolaAc;
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
    inline void reset(const Eigen::Matrix<double, 3, 2> &headState,
                      const Eigen::Matrix<double, 3, 2> &tailState,
                      const int &pieceNum)
    {
        N = pieceNum;
        headPV = headState;
        tailPV = tailState;
        T1.resize(N);
        A.create(4 * N, 4, 4);
        b.resize(4 * N, 3);
        gdC.resize(4 * N, 3);
        return;
    }

    inline void generate(const Eigen::MatrixXd &inPs,
                         const Eigen::VectorXd &ts)
    {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);
        T3 = T2.cwiseProduct(T1);

        A.reset();
        b.setZero();

        A(0, 0) = 1.0;
        A(1, 1) = 1.0;
        b.row(0) = headPV.col(0).transpose();
        b.row(1) = headPV.col(1).transpose();

        for (int i = 0; i < N - 1; i++)
        {
            A(4 * i + 2, 4 * i + 2) = 2.0;
            A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
            A(4 * i + 2, 4 * i + 6) = -2.0;
            A(4 * i + 3, 4 * i) = 1.0;
            A(4 * i + 3, 4 * i + 1) = T1(i);
            A(4 * i + 3, 4 * i + 2) = T2(i);
            A(4 * i + 3, 4 * i + 3) = T3(i);
            A(4 * i + 4, 4 * i) = 1.0;
            A(4 * i + 4, 4 * i + 1) = T1(i);
            A(4 * i + 4, 4 * i + 2) = T2(i);
            A(4 * i + 4, 4 * i + 3) = T3(i);
            A(4 * i + 4, 4 * i + 4) = -1.0;
            A(4 * i + 5, 4 * i + 1) = 1.0;
            A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
            A(4 * i + 5, 4 * i + 3) = 3.0 * T2(i);
            A(4 * i + 5, 4 * i + 5) = -1.0;

            b.row(4 * i + 3) = inPs.col(i).transpose();
        }

        A(4 * N - 2, 4 * N - 4) = 1.0;
        A(4 * N - 2, 4 * N - 3) = T1(N - 1);
        A(4 * N - 2, 4 * N - 2) = T2(N - 1);
        A(4 * N - 2, 4 * N - 1) = T3(N - 1);
        A(4 * N - 1, 4 * N - 3) = 1.0;
        A(4 * N - 1, 4 * N - 2) = 2 * T1(N - 1);
        A(4 * N - 1, 4 * N - 1) = 3 * T2(N - 1);

        b.row(4 * N - 2) = tailPV.col(0).transpose();
        b.row(4 * N - 1) = tailPV.col(1).transpose();

        A.factorizeLU();
        A.solve(b);

        return;
    }

    inline double getTrajAccCost() const
    {
        double objective = 0.0;
        for (int i = 0; i < N; i++)
        {
            objective += 4.0 * b.row(4 * i + 2).squaredNorm() * T1(i) +
                         12.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T2(i) +
                         12.0 * b.row(4 * i + 3).squaredNorm() * T3(i);
        }
        return objective;
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

        cost = getTrajAccCost();
        addGradJbyT(gdT);
        addGradJbyC(gdC);

        addTimeIntPenalty(cons, idxHs, cfgHs, vmax, amax, ci, cost, gdT, gdC);

        solveAdjGradC(gdC);
        addPropCtoT(gdC, gdT);
        addPropCtoP(gdC, gdInPs);
    }

    inline Trajectory getTraj(void) const
    {
        Trajectory traj;
        traj.reserve(N);
        for (int i = 0; i < N; i++)
        {
            traj.emplace_back(T1(i), b.block<4, 3>(4 * i, 0).transpose().rowwise().reverse());
        }
        return traj;
    }
};

// class GCOPTER
// {
// private:
//     // Use C2 or Cinf diffeo
//     bool c2dfm;

//     // Use soft time or not
//     bool softT;

//     // Weight for time regularization term
//     double rho;

//     // Fixed total time
//     double sumT;

//     //Minimum Acc Optimizer
//     MinAccOpt accOpt;

//     // Temp variables for problem solving
//     Eigen::MatrixXd iState;
//     Eigen::MatrixXd fState;

//     // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
//     std::vector<Eigen::MatrixXd> cfgVs;
//     std::vector<Eigen::MatrixXd> cfgHs;
//     Eigen::MatrixXd gdInPs;

//     // Piece num for each polytope
//     Eigen::VectorXi intervals;
//     // Assignment vector for point in V-polytope
//     Eigen::VectorXi idxVs;
//     // Assignment vector for piece in H-polytope
//     Eigen::VectorXi idxHs;

//     int coarseN;
//     int fineN;
//     int dimFreeT;
//     int dimFreeP;
//     Eigen::VectorXd coarseT;
//     Eigen::VectorXd fineT;
//     Eigen::MatrixXd innerP;

//     // Params for constraints
//     Eigen::VectorXi cons;
//     Eigen::Vector3d chi;
//     double vmax;
//     double amax;

//     // L-BFGS Solver Parameters
//     lbfgs::lbfgs_parameter_t lbfgs_params;

// private:
//     template <typename EIGENVEC>
//     static inline void forwardT(const EIGENVEC &t,
//                                 Eigen::VectorXd &vecT,
//                                 bool soft,
//                                 const double &sT,
//                                 bool c2)
//     {
//         if (soft)
//         {
//             if (c2)
//             {
//                 int M = vecT.size();
//                 for (int i = 0; i < M; i++)
//                 {
//                     vecT(i) = t(i) > 0.0
//                                   ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
//                                   : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
//                 }
//             }
//             else
//             {
//                 vecT = t.array().exp();
//             }
//         }
//         else
//         {
//             if (c2)
//             {
//                 int Ms1 = t.size();
//                 for (int i = 0; i < Ms1; i++)
//                 {
//                     vecT(i) = t(i) > 0.0
//                                   ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
//                                   : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
//                 }
//                 vecT(Ms1) = 0.0;
//                 vecT /= 1.0 + vecT.sum();
//                 vecT(Ms1) = 1.0 - vecT.sum();
//                 vecT *= sT;
//             }
//             else
//             {
//                 int Ms1 = t.size();
//                 vecT.head(Ms1) = t.array().exp();
//                 vecT(Ms1) = 0.0;
//                 vecT /= 1.0 + vecT.sum();
//                 vecT(Ms1) = 1.0 - vecT.sum();
//                 vecT *= sT;
//             }
//         }
//         return;
//     }

//     template <typename EIGENVEC>
//     static inline void backwardT(const Eigen::VectorXd &vecT,
//                                  EIGENVEC &t,
//                                  bool soft,
//                                  bool c2)
//     {
//         if (soft)
//         {
//             if (c2)
//             {
//                 int M = vecT.size();
//                 for (int i = 0; i < M; i++)
//                 {
//                     t(i) = vecT(i) > 1.0
//                                ? (sqrt(2.0 * vecT(i) - 1.0) - 1.0)
//                                : (1.0 - sqrt(2.0 / vecT(i) - 1.0));
//                 }
//             }
//             else
//             {
//                 t = vecT.array().log();
//             }
//         }
//         else
//         {
//             if (c2)
//             {
//                 int Ms1 = t.size();
//                 t = vecT.head(Ms1) / vecT(Ms1);
//                 for (int i = 0; i < Ms1; i++)
//                 {
//                     t(i) = t(i) > 1.0
//                                ? (sqrt(2.0 * t(i) - 1.0) - 1.0)
//                                : (1.0 - sqrt(2.0 / t(i) - 1.0));
//                 }
//             }
//             else
//             {
//                 int Ms1 = t.size();
//                 t = (vecT.head(Ms1) / vecT(Ms1)).array().log();
//             }
//         }
//         return;
//     }

//     template <typename EIGENVEC>
//     static inline void forwardP(const EIGENVEC &p,
//                                 const Eigen::VectorXi &idVs,
//                                 const std::vector<Eigen::MatrixXd> &cfgPolyVs,
//                                 Eigen::MatrixXd &inP)
//     {
//         int M = inP.cols();
//         Eigen::VectorXd q;
//         int j = 0, k, idx;
//         for (int i = 0; i < M; i++)
//         {
//             idx = idVs(i);
//             k = cfgPolyVs[idx].cols() - 1;
//             q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
//             inP.col(i) = cfgPolyVs[idx].rightCols(k) * q.cwiseProduct(q) +
//                          cfgPolyVs[idx].col(0);
//             j += k;
//         }
//         return;
//     }

//     static inline double objectiveNLS(void *ptrPOBs,
//                                       const double *x,
//                                       double *grad,
//                                       const int n)
//     {
//         const Eigen::MatrixXd &pobs = *(Eigen::MatrixXd *)ptrPOBs;
//         Eigen::Map<const Eigen::VectorXd> p(x, n);
//         Eigen::Map<Eigen::VectorXd> gradp(grad, n);

//         double qnsqr = p.squaredNorm();
//         double qnsqrp1 = qnsqr + 1.0;
//         double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
//         Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

//         Eigen::Vector3d delta = pobs.rightCols(n) * r.cwiseProduct(r) +
//                                 pobs.col(1) - pobs.col(0);
//         double cost = delta.squaredNorm();
//         Eigen::Vector3d gradR3 = 2 * delta;

//         Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
//         gdr = gdr.array() * r.array() * 2.0;
//         gradp = gdr * 2.0 / qnsqrp1 -
//                 p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

//         return cost;
//     }

//     template <typename EIGENVEC>
//     static inline void backwardP(const Eigen::MatrixXd &inP,
//                                  const Eigen::VectorXi &idVs,
//                                  const std::vector<Eigen::MatrixXd> &cfgPolyVs,
//                                  EIGENVEC &p)
//     {
//         int M = inP.cols();
//         int j = 0, k, idx;

//         // Parameters for tiny nonlinear least squares
//         double minSqrD;
//         lbfgs::lbfgs_parameter_t nls_params;
//         lbfgs::lbfgs_load_default_parameters(&nls_params);
//         nls_params.g_epsilon = FLT_EPSILON;
//         nls_params.max_iterations = 128;

//         Eigen::MatrixXd pobs;
//         for (int i = 0; i < M; i++)
//         {
//             idx = idVs(i);
//             k = cfgPolyVs[idx].cols() - 1;
//             p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
//             pobs.resize(3, k + 2);
//             pobs.col(0) = inP.col(i);
//             pobs.rightCols(k + 1) = cfgPolyVs[idx];
//             lbfgs::lbfgs_optimize(k,
//                                   p.data() + j,
//                                   &minSqrD,
//                                   &GCOPTER::objectiveNLS,
//                                   nullptr,
//                                   nullptr,
//                                   &pobs,
//                                   &nls_params);

//             j += k;
//         }

//         return;
//     }

//     template <typename EIGENVEC>
//     static inline void addLayerTGrad(const Eigen::VectorXd &t,
//                                      EIGENVEC &gradT,
//                                      bool soft,
//                                      const double &sT,
//                                      bool c2)
//     {
//         if (soft)
//         {
//             if (c2)
//             {
//                 int M = t.size();
//                 double denSqrt;
//                 for (int i = 0; i < M; i++)
//                 {
//                     if (t(i) > 0)
//                     {
//                         gradT(i) *= t(i) + 1.0;
//                     }
//                     else
//                     {
//                         denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
//                         gradT(i) *= (1.0 - t(i)) / (denSqrt * denSqrt);
//                     }
//                 }
//             }
//             else
//             {
//                 int M = t.size();
//                 gradT.head(M).array() *= t.array().exp();
//             }
//         }
//         else
//         {
//             if (c2)
//             {
//                 int Ms1 = t.size();
//                 Eigen::VectorXd gFree = sT * gradT.head(Ms1);
//                 double gTail = sT * gradT(Ms1);
//                 Eigen::VectorXd dExpTau(Ms1);
//                 double expTauSum = 0.0, gFreeDotExpTau = 0.0;
//                 double denSqrt, expTau;
//                 for (int i = 0; i < Ms1; i++)
//                 {
//                     if (t(i) > 0)
//                     {
//                         expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
//                         dExpTau(i) = t(i) + 1.0;
//                         expTauSum += expTau;
//                         gFreeDotExpTau += expTau * gFree(i);
//                     }
//                     else
//                     {
//                         denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
//                         expTau = 1.0 / denSqrt;
//                         dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
//                         expTauSum += expTau;
//                         gFreeDotExpTau += expTau * gFree(i);
//                     }
//                 }
//                 denSqrt = expTauSum + 1.0;
//                 gradT.head(Ms1) = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
//                                   (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
//                 gradT(Ms1) = 0.0;
//             }
//             else
//             {
//                 int Ms1 = t.size();
//                 Eigen::VectorXd gFree = sT * gradT.head(Ms1);
//                 double gTail = sT * gradT(Ms1);
//                 Eigen::VectorXd expTau = t.array().exp();
//                 double expTauSum = expTau.sum();
//                 double denom = expTauSum + 1.0;
//                 gradT.head(Ms1) = (gFree.array() - gTail) * expTau.array() / denom -
//                                   (gFree.dot(expTau) - gTail * expTauSum) * expTau.array() / (denom * denom);
//                 gradT(Ms1) = 0.0;
//             }
//         }
//         return;
//     }

//     template <typename EIGENVEC_0, typename EIGENVEC_1>
//     static inline void addLayerPGrad(EIGENVEC_0 &p,
//                                      const Eigen::VectorXi &idVs,
//                                      const std::vector<Eigen::MatrixXd> &cfgPolyVs,
//                                      const Eigen::MatrixXd &gradInPs,
//                                      EIGENVEC_1 &grad)
//     {
//         int M = gradInPs.cols();

//         int j = 0, k, idx;
//         double qnsqr, qnsqrp1, qnsqrp1sqr;
//         Eigen::VectorXd q, r, gdr;
//         for (int i = 0; i < M; i++)
//         {
//             idx = idVs(i);
//             k = cfgPolyVs[idx].cols() - 1;

//             q = p.segment(j, k);
//             qnsqr = q.squaredNorm();
//             qnsqrp1 = qnsqr + 1.0;
//             qnsqrp1sqr = qnsqrp1 * qnsqrp1;
//             r = 2.0 / qnsqrp1 * q;
//             gdr = cfgPolyVs[idx].rightCols(k).transpose() * gradInPs.col(i);
//             gdr = gdr.array() * r.array() * 2.0;

//             grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
//                                  q * 4.0 * gdr.dot(q) / qnsqrp1sqr;

//             j += k;
//         }

//         return;
//     }

//     static inline void splitToFineT(const Eigen::VectorXd &cT,
//                                     const Eigen::VectorXi &intervs,
//                                     Eigen::VectorXd &fT)
//     {
//         int M = intervs.size();
//         int offset = 0;
//         int inverv;
//         for (int i = 0; i < M; i++)
//         {
//             inverv = intervs(i);
//             fT.segment(offset, inverv).setConstant(cT(i) / inverv);
//             offset += inverv;
//         }
//         return;
//     }

//     static inline void mergeToCoarseGradT(const Eigen::VectorXi &intervs,
//                                           Eigen::VectorXd &fineGdT)
//     {
//         int M = intervs.size();
//         int offset = 0;
//         int inverv;
//         for (int i = 0; i < M; i++)
//         {
//             inverv = intervs(i);
//             fineGdT(i) = fineGdT.segment(offset, inverv).mean();
//             offset += inverv;
//         }
//         return;
//     }

//     static inline double objectiveFunc(void *ptrObj,
//                                        const double *x,
//                                        double *grad,
//                                        const int n)
//     {
//         GCOPTER &obj = *(GCOPTER *)ptrObj;
//         const int dimT = obj.dimFreeT;
//         const int dimP = obj.dimFreeP;
//         const double rh = obj.rho;
//         Eigen::Map<const Eigen::VectorXd> t(x, dimT);
//         Eigen::Map<const Eigen::VectorXd> p(x + dimT, dimP);
//         Eigen::Map<Eigen::VectorXd> gradt(grad, dimT);
//         Eigen::VectorXd proxyGradT(obj.fineN);
//         Eigen::Map<Eigen::VectorXd> gradp(grad + dimT, dimP);

//         forwardT(t, obj.coarseT, obj.softT, obj.sumT, obj.c2dfm);
//         splitToFineT(obj.coarseT, obj.intervals, obj.fineT);
//         forwardP(p, obj.idxVs, obj.cfgVs, obj.innerP);

//         double cost;

//         obj.accOpt.generate(obj.innerP, obj.fineT);
//         obj.accOpt.evalTrajCostGrad(obj.cons,
//                                     obj.idxHs, obj.cfgHs,
//                                     obj.vmax, obj.amax,
//                                     obj.chi, cost,
//                                     proxyGradT, obj.gdInPs);

//         cost += rh * obj.coarseT.sum();
//         proxyGradT.array() += rh;

//         mergeToCoarseGradT(obj.intervals, proxyGradT);
//         addLayerTGrad(t, proxyGradT, obj.softT, obj.sumT, obj.c2dfm);
//         addLayerPGrad(p, obj.idxVs, obj.cfgVs, obj.gdInPs, gradp);

//         gradt = proxyGradT.head(dimT);

//         return cost;
//     }

// public:
//     inline void gridMesh(const Eigen::Matrix<double, 3, 2> &iState,
//                          const Eigen::Matrix<double, 3, 2> &fState,
//                          const std::vector<Eigen::MatrixXd> &cfgPolyVs,
//                          const double &gridResolution,
//                          Eigen::VectorXi &intervalsVec) const
//     {
//         int M = intervalsVec.size();

//         int curInterval, k;
//         Eigen::Vector3d lastP, curP;
//         curP = iState.col(0);
//         for (int i = 0; i < M - 1; i++)
//         {
//             lastP = curP;
//             k = cfgPolyVs[2 * i + 1].cols() - 1;
//             curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
//                    cfgPolyVs[2 * i + 1].col(0);
//             curInterval = ceil((curP - lastP).norm() / gridResolution);
//             intervalsVec(i) = curInterval > 0 ? curInterval : 1;
//         }
//         lastP = curP;
//         curP = fState.col(0);
//         curInterval = ceil((curP - lastP).norm() / gridResolution);
//         intervalsVec(M - 1) = curInterval > 0 ? curInterval : 1;

//         return;
//     }

//     inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
//                           std::vector<Eigen::MatrixXd> &vPs) const
//     {
//         const int M = hPs.size() - 1;

//         vPs.clear();
//         vPs.reserve(2 * M + 1);

//         int nv;
//         Eigen::MatrixXd curIH, curIV, curIOB;
//         for (int i = 0; i < M; i++)
//         {
//             if (!geoutils::enumerateVs(hPs[i], curIV))
//             {
//                 return false;
//             }
//             nv = curIV.cols();
//             curIOB.resize(3, nv);
//             curIOB.col(0) = curIV.col(0);
//             curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
//             vPs.push_back(curIOB);

//             curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
//             curIH.leftCols(hPs[i].cols()) = hPs[i];
//             curIH.rightCols(hPs[i + 1].cols()) = hPs[i + 1];
//             if (!geoutils::enumerateVs(curIH, curIV))
//             {
//                 return false;
//             }
//             nv = curIV.cols();
//             curIOB.resize(3, nv);
//             curIOB.col(0) = curIV.col(0);
//             curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
//             vPs.push_back(curIOB);
//         }

//         if (!geoutils::enumerateVs(hPs.back(), curIV))
//         {
//             return false;
//         }
//         nv = curIV.cols();
//         curIOB.resize(3, nv);
//         curIOB.col(0) = curIV.col(0);
//         curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
//         vPs.push_back(curIOB);

//         return true;
//     }

//     inline bool setup(const double &rh,
//                       const double &st,
//                       const Eigen::MatrixXd &iniState,
//                       const Eigen::MatrixXd &finState,
//                       const std::vector<Eigen::MatrixXd> &cfgPolyHs,
//                       const double &gridRes,
//                       const int &itgSpaces,
//                       const double &vm,
//                       const double &am,
//                       const Eigen::Vector3d &w,
//                       bool c2diffeo)
//     {
//         // Setup for optimization parameters
//         c2dfm = c2diffeo;

//         softT = rh > 0;
//         if (softT)
//         {
//             rho = rh;
//             sumT = 1.0;
//         }
//         else
//         {
//             rho = 0.0;
//             sumT = st;
//         }

//         iState = iniState;
//         fState = finState;

//         cfgHs = cfgPolyHs;
//         coarseN = cfgHs.size();
//         for (int i = 0; i < coarseN; i++)
//         {
//             cfgHs[i].topRows<3>().colwise().normalize();
//         }
//         if (!extractVs(cfgHs, cfgVs))
//         {
//             return false;
//         }

//         intervals.resize(coarseN);
//         gridMesh(iState, fState, cfgVs, gridRes, intervals);
//         fineN = intervals.sum();
//         cons.resize(fineN);
//         cons.setConstant(itgSpaces);

//         idxVs.resize(fineN - 1);
//         idxHs.resize(fineN);
//         dimFreeT = softT ? coarseN : coarseN - 1;
//         dimFreeP = 0;
//         int offset = 0, interval;
//         for (int i = 0; i < coarseN; i++)
//         {
//             interval = intervals(i);
//             for (int j = 0; j < interval; j++)
//             {
//                 if (j < interval - 1)
//                 {
//                     idxVs(offset) = 2 * i;
//                     dimFreeP += cfgVs[2 * i].cols() - 1;
//                 }
//                 else if (i < coarseN - 1)
//                 {
//                     idxVs(offset) = 2 * i + 1;
//                     dimFreeP += cfgVs[2 * i + 1].cols() - 1;
//                 }
//                 idxHs(offset) = i;
//                 offset++;
//             }
//         }

//         chi = w;
//         vmax = vm;
//         amax = am;

//         // Make all conditions legal
//         double tempNorm;
//         tempNorm = iState.col(1).norm();
//         iState.col(1) *= tempNorm > vmax ? (vmax / tempNorm) : 1.0;
//         tempNorm = fState.col(1).norm();
//         fState.col(1) *= tempNorm > vmax ? (vmax / tempNorm) : 1.0;

//         // Setup for L-BFGS solver
//         lbfgs::lbfgs_load_default_parameters(&lbfgs_params);

//         // Allocate temp variables
//         coarseT.resize(coarseN);
//         fineT.resize(fineN);
//         innerP.resize(3, fineN - 1);
//         gdInPs.resize(3, fineN - 1);
//         accOpt.reset(iniState, finState, fineN);

//         return true;
//     }

//     inline void setInitial(const std::vector<Eigen::MatrixXd> &cfgPolyVs,
//                            const Eigen::VectorXi &intervs,
//                            Eigen::VectorXd &vecT,
//                            Eigen::MatrixXd &vecInP) const
//     {
//         constexpr double maxSpeedForAllocatiion = 10.0;

//         int M = vecT.size();
//         Eigen::Vector3d lastP, curP, delta;
//         int offset, interv, k;

//         offset = 0;
//         curP = iState.col(0);
//         for (int i = 0; i < M - 1; i++)
//         {
//             lastP = curP;
//             interv = intervs(i);
//             k = cfgPolyVs[2 * i + 1].cols() - 1;
//             curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
//                    cfgPolyVs[2 * i + 1].col(0);
//             delta = curP - lastP;
//             vecT(i) = delta.norm() / std::min(vmax, maxSpeedForAllocatiion);
//             delta /= interv;
//             for (int j = 0; j < interv; j++)
//             {
//                 vecInP.col(offset++) = (j + 1) * delta + lastP;
//             }
//         }
//         interv = intervs(M - 1);
//         lastP = curP;
//         curP = fState.col(0);
//         delta = curP - lastP;
//         vecT(M - 1) = delta.norm() / std::min(vmax, maxSpeedForAllocatiion);
//         delta /= interv;
//         for (int j = 0; j < interv - 1; j++)
//         {
//             vecInP.col(offset++) = (j + 1) * delta + lastP;
//         }

//         return;
//     }

//     inline double optimize(Trajectory &traj,
//                            const double &relCostTol)
//     {
//         double *x = new double[dimFreeT + dimFreeP];
//         Eigen::Map<Eigen::VectorXd> t(x, dimFreeT);
//         Eigen::Map<Eigen::VectorXd> p(x + dimFreeT, dimFreeP);

//         setInitial(cfgVs, intervals, coarseT, innerP);

//         backwardT(coarseT, t, softT, c2dfm);
//         backwardP(innerP, idxVs, cfgVs, p);

//         double minObjectivePenalty;
//         lbfgs_params.mem_size = 64;
//         lbfgs_params.past = 3;
//         lbfgs_params.g_epsilon = 1.0e-16;
//         lbfgs_params.min_step = 1.0e-32;
//         lbfgs_params.delta = relCostTol;

//         int retval = lbfgs::lbfgs_optimize(dimFreeT + dimFreeP,
//                                            x,
//                                            &minObjectivePenalty,
//                                            &GCOPTER::objectiveFunc,
//                                            nullptr,
//                                            nullptr,
//                                            this,
//                                            &lbfgs_params);
//         std::cout << lbfgs::lbfgs_strerror(retval) << std::endl;

//         forwardT(t, coarseT, softT, sumT, c2dfm);
//         splitToFineT(coarseT, intervals, fineT);
//         forwardP(p, idxVs, cfgVs, innerP);

//         accOpt.generate(innerP, fineT);
//         traj = accOpt.getTraj();

//         delete[] x;

//         return accOpt.getTrajAccCost();
//     }
// };

#endif
