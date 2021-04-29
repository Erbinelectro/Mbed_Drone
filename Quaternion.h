#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include<math.h>

/**
* クォータニオンの足し，引き，掛け算などを簡単にできるようになります．
* @author  Gaku MATSUMOTO
* @bref  クォータニオンを使えるクラスです．
*/
class Quaternion {
public:
	/**
		@bref	Quaternionインスタンスを生成します
	*/
	Quaternion() {
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	};
	/**
		@bref	要素を代入しながら，インスタンスを生成します．
		@param[in]  _w  実部wの初期値
		@param[in]  _x  虚部iの初期値
		@param[in]  _y  虚部jの初期値
		@param[in]  _z  虚部kの初期値
	*/
	Quaternion(double _w, double _x, double _y, double _z) {
		w = _w; x = _x; y = _y; z = _z;
	};

public:
	double w;
	double x;
	double y;
	double z;

public:

	/**
		@bref  クォータニオンの要素をコピーします．
		@note  通常の数のように代入できます
	*/
	Quaternion operator=(Quaternion r) {
		w = r.w;
		x = r.x;
		y = r.y;
		z = r.z;
		return *this;
	};

	/**
		@bref  クォータニオンを足して代入します．
		@note  通常の数のように代入できます
	*/
	Quaternion operator+=(Quaternion r) {
		w += r.w;
		x += r.x;
		y += r.y;
		z += r.z;
		return *this;
	};

	/**
		@bref  クォータニオンを引いて代入します．
		@note  通常の数のように代入できます
	*/
	Quaternion operator-=(Quaternion r) {
		w -= r.w;
		x -= r.x;
		y -= r.y;
		z -= r.z;
		return *this;
	};

	/**
	*	@bref	クォータニオンの掛け算をします．
	*	@note	この際も順序は重要です．
	*/
	Quaternion operator*=(Quaternion r) {
		static Quaternion QQ;
		QQ.w = w * r.x - x * r.x - y * r.y - z * r.z;
		QQ.x = x * r.w + w * r.x - z * r.y + y * r.z;
		QQ.y = y * r.w + z * r.x + w * r.y - x * r.z;
		QQ.z = z * r.w - y * r.x + x * r.y + w * r.z;
		w = QQ.w;
		x = QQ.x;
		y = QQ.y;
		z = QQ.z;
		return *this;
	};

	/**
		@bref	クォータニオンの複素共役を返します．
		@note	本当はアスタリスクが良かったのですが，ポインタと紛らわしいのでマイナスにしました．
	*/
	Quaternion operator-() {
		Quaternion Q;
		Q.w = w;
		Q.x = -x;
		Q.y = -y;
		Q.z = -z;
		return Q;
	};

	/**
		@bref	クォータニオンを正規化して，単位クォータニオンにします．
		@note	掛け算などを行うたびに実行することをお勧めします．
	*/
	void normalize() {
		double norm = sqrt(w * w + x * x + y * y + z * z);
		if (norm != 0.0) {
			w /= norm;
			x /= norm;
			y /= norm;
			z /= norm;
			return;
		}
		else {
			return;
		}
	};
};

/**
* @fn Quaternion operator*(Quaternion l, Quaternion r)
* @bref クォータニオンの掛け算をします．この際，順序が重要です．
*/
Quaternion operator*(Quaternion l, Quaternion r) {
	static Quaternion Q;
	Q.w = l.w * r.w - l.x * r.x - l.y * r.y - l.z * r.z;
	Q.x = l.x * r.w + l.w * r.x - l.z * r.y + l.y * r.z;
	Q.y = l.y * r.w + l.z * r.x + l.w * r.y - l.x * r.z;
	Q.z = l.z * r.w - l.y * r.x + l.x * r.y + l.w * r.z;
	/*double q0Dot, q1Dot, q2Dot, q3Dot;//クォータニオンの時間微分
	q0Dot =        -gx*qg1 - gy*qg2 - gz*qg3;
	q1Dot = gx*qg0         + gz*qg2 - gy*qg3;
	q2Dot = gy*qg0 - gz*qg1         + gx*qg3;
	q3Dot = gz*qg0 + gy*qg1 - gx*qg2;*/
	return Q;
};

/**
* @fn Quaternion operator*(double s, Quaternion q)
* @bref クォータニオンをスカラー倍します．．
*/
Quaternion operator*(double s, Quaternion q) {
	static Quaternion Q;
	Q.w = q.w * s;
	Q.x = q.x * s;
	Q.y = q.y * s;
	Q.z = q.z * s;
	return Q;
};

/**
* @fn Quaternion operator*(Quaternion q, double s)
* @bref クォータニオンをスカラー倍します．．
*/
Quaternion operator*(Quaternion q, double s) {
	static Quaternion Q;
	Q.w = q.w * s;
	Q.x = q.x * s;
	Q.y = q.y * s;
	Q.z = q.z * s;
	return Q;
};

/**
* @fn Quaternion operator+(Quaternion l, Quaternion r)
* @bref クォータニオンの足し算をします．
*/
Quaternion operator+(Quaternion l, Quaternion r) {
	static Quaternion Q;
	Q.w = l.w + r.w;
	Q.x = l.x + r.x;
	Q.y = l.y + r.y;
	Q.z = l.z + r.z;
	return Q;
}

/**
* @fn Quaternion operator-(Quaternion l, Quaternion r)
* @bref クォータニオンの引き算をします．
*/
Quaternion operator-(Quaternion l, Quaternion r) {
	static Quaternion Q;
	Q.w = l.w - r.w;
	Q.x = l.x - r.x;
	Q.y = l.y - r.y;
	Q.z = l.z - r.z;
	return Q;
}

#endif