#include<iostream>
#include<vector>
#include<tuple>
#include<algorithm>
#include<math.h>
#include<string>
#include<stack>
#include<queue>
#include<stdio.h>
#include<iomanip>
#include<functional>
#include<set>
#include<map>
#include<bitset>
#include<random>
using namespace std;
//変数など
#define rep1(i, a) for(int i = 0;i < a; i++)
#define rep2(i, a, b) for(int i = a;i < b;i++)
#define rep3(i, b) for(int i = b;i >= 0;i--)
#define rep4(i, a, b) for(int i = a;i >= b;i--)
using ll = long long;
using ull = unsigned long long int;
using pii = pair<int, int>;
using pll = pair<ll, ll>;
using ld = long double;
template <class T> using vc = vector<T>;
template <class T> using vvc = vector<vc<T>>;
template <class T> using vvvc = vector<vvc<T>>;
using vi = vc<int>;
using vvi = vvc<int>;
using vvvi = vvvc<int>;
using vl = vc<ll>;
using vvl = vvc<ll>;
using vvvl = vvvc<ll>;
using vpi = vc<pii>;
using vvpi = vvc<pii>;
using vvvpi = vvvc<pii>;
using vpl = vc<pll>;
using vvpl = vvc<pll>;
using vvvpl = vvvc<pll>;
template <class T> using pq = priority_queue<T>;
template <class T> using pqg = priority_queue<T, vector<T>, greater<T>>;
//定数
const long long INF_ll = 1LL<<59;
const int INF_int = 1<<29;
const double ESP = 1e-10;
//関数
template <class T> bool chmax(T &a, T b) {
    if(b > a) {a = b; return true;}
    return false;
    }
template <class T> bool chmin(T &a, T b) {
    if(b < a) {a = b;return true;}
    return false;
    }
template <class T> T gcd(T a, T b) {
    if(b == 0) return a;
    return gcd(b, a%b);
}
template <class T> T lcm(T a, T b) {
    return a * (b /gcd(a, b));
}
template <class T> T extgcd(T a, T b, T& x, T& y){
    T d = a;
    if(b != 0){
        d = extgcd(b, a%b, y, x);
        y -= x*(a/b);
    }
    else {
        x = 1;
        y = 0;
    }
    return d;
}
template <class T> T modinv(T a, T M){
    T x, y;
    extgcd(a, M, x, y);
    x %= M;
    if(x < 0) x += M;
    return x;
}
double add(double a, double b){
    if(abs(a+b) < ESP * (abs(a)+abs(b))) return 0;
    return a+b;
}
//構造体
struct P{  //ベクトル
    double x, y;
    P(){}
    P(double a, double b) : x(a), y(b) {}
    P operator + (P p) {
        return P(add(x, p.x), add(y, p.y));
    }
    P operator - (P p) {
        return P(add(x, -p.x), add(y, -p.y));
    }
    P operator * (double d){
        return P(x * d, y * d);
    }
    double dot(P p){  //内積
        return add(x * p.x, y * p.y);
    }
    double det(P p){  //外積
        return add(x * p.y, -y * p.x);
    }
};
struct UnionFind
{
    vector<int> par, siz, w;
    UnionFind(int n) : par(n, -1), siz(n, 1) {}
    int root(int x)
    {
        if(par[x] == -1) return x;
        else return par[x] = root(par[x]);
    }
    bool issame(int x, int y)
    {
        return root(x) == root(y);
    }
    bool unite(int x, int y)
    {
        x = root(x);
        y = root(y);
        if(x == y) return false;
        if(siz[x] < siz[y]) swap(x, y);
        par[y] = x;
        siz[x] += siz[y];
        return true;
    }
    int size(int x)
    {
        return siz[root(x)];
    }
};
template <class T> struct Segment_tree {
    private:
        int n; 
        vector<T> dat, lazy;
        vector<bool> isLazy;
        T def;
        function<T(T, T)> operation, update;
    public : Segment_tree(vector<T> Data, T def_, function<T(T, T)> operation_, function<T(T, T)> update_ = [](T a, T b){return b;}) : 
    def(def_), operation(operation_), update(update_)
    {
        n = 1;
        while(n < int(Data.size())) n *= 2;
        dat.resize(2*n);
        lazy.resize(2*n);
        isLazy.resize(2*n, false);
        rep2(i, n, 2*n){
            if(int(Data.size()) > i-n) dat[i] = Data[i-n];
            else dat[i] = def;
        }
        init();
    }
    private : void init(){
        rep4(i,n-1,1) dat[i] = operation(dat[i*2], dat[i*2+1]);
    }
    private : void eval(int v){
        if(!isLazy[v]) return;
        if(v < n){
            if(isLazy[2*v]) lazy[2*v] = update(lazy[2*v], lazy[v]);
            else lazy[2*v] = lazy[v];
            if(isLazy[2*v+1]) lazy[2*v+1] = update(lazy[2*v+1], lazy[v]);
            else lazy[2*v+1] = lazy[v];
            isLazy[2*v] = isLazy[2*v+1] = true;
        }
        dat[v] = update(dat[v], lazy[v]);
        isLazy[v] = false;
    }
    public : void RUQ(int a, int b, T x, int v = 1, int l = 0, int r = -1){
        if(r == -1) r = n;
        eval(v);
        if(r <= a || l >= b) return;
        else if(a <= l && r <= b) {
            lazy[v] = x;
            isLazy[v] = true;
            eval(v);
        }
        else {
            if(v < n){
                RUQ(a, b, x, v*2+0, l, (l+r)/2);
                RUQ(a, b, x, v*2+1, (l+r)/2, r);
                dat[v] = operation(dat[v*2], dat[v*2+1]);
            }
        }
    }
    public : T prod(int a, int b, int v = 1, int l = 0, int r = -1)
    {
        eval(v);
        if(r == -1) r = n;
        if(r <= a || l >= b) return def;
        if(a <= l && r <= b) return dat[v];
        else{
            return operation(prod(a, b, v*2, l, (l+r)/2), prod(a, b, v*2+1, (l+r)/2, r));
        }
    }
    public : void Update(int x, T a){//RUQと同時には使わない
        x += n;
        dat[x] = a;
        while(x > 1){
            x >>= 1;
            dat[x] = operation(dat[x*2], dat[x*2+1]);
        }
    }
    public : T upper_bound(T a){
        T sum = def;
        int v = 1;
        while(v<n){
            if(operation(sum, dat[v*2])>a){
                v *= 2;
            } 
            else {//右に進む
                sum = operation(sum, dat[2*v]);
                v = v*2+1;
            }
        }
        sum = operation(sum,dat[v]);
        return sum;
    }
};
template <class T> struct BIT{
    vector<T> bit;
    BIT(vector<T> x){
        rep2(i, 1, int(x.size())) x[i] += x[i-1];
        bit.resize(x.size()+1);
        bit[0] = 0;
        rep2(i, 1, int(x.size())+1) bit[i] = x[i-1];
        rep4(i, int(x.size()), 1){
            bit[i] -= bit[i- (i & -i)];
        }
    }
    T sum(int i){
        i++;
        T s = 0;
        while(i > 0){
            s += bit[i];
            i -= i & -i;
        }
        return s;
    }
    void add(int i, T x){
        i++;
        while(i < int(bit.size())){
            bit[i] += x;
            i += i & -i;
        }
    }
};
struct fraction {
    public : ll mother, son;
    fraction(ll a, ll b) : mother(b), son(a) {}//分子, 分母の準
    fraction() : mother(1), son(0) {}
    fraction operator+(fraction x){
        fraction a;
        a.mother = mother*x.mother;
        a.son = son * x.mother + mother * x.son;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator-(fraction x){
        fraction a;
        a.mother = mother*x.mother;
        a.son = son * x.mother - mother * x.son;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator*(fraction x){
        fraction a;
        a.mother = mother*x.mother;
        a.son = son * x.son;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator/(fraction x){
        fraction a;
        a.mother = mother*x.son;
        a.son = son * x.mother;
        if(a.mother<0) {a.mother *= -1; a.son *= -1;}
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator+(ll x){
        fraction a;
        a.mother = mother;
        a.son = son + mother * x;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator-(ll x){
        fraction a;
        a.mother = mother;
        a.son = son - mother * x;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator*(ll x){
        fraction a;
        a.mother = mother;
        a.son = son * x;
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction operator/(ll x){
        fraction a;
        a.mother = mother*x;
        a.son = son;
        if(a.mother<0) {a.mother *= -1; a.son *= -1;}
        ll g = gcd(abs(a.son), a.mother);
        a.son /= g; a.mother /= g;
        return a;
    }
    fraction reciprocal(){//0の場合は0/1を返す
        if(son == 0) return fraction(0, 1);
        else return fraction(mother, son);
    }
    string out(){
        string S;
        if(mother==1) S = to_string(son);
        else S = to_string(son) + "/" + to_string(mother);
        return S;
    }
};




int main()
{
    cout << setprecision(15);
    ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    //std:: random_device rnd;
    ll H,W,X,P,Q; cin >> H >> W >> X >> P >> Q;
    P--;Q--;
    vvl A(H,vl(W));rep1(i,H) rep1(j,W) cin >> A[i][j];
    ll ans = A[P][Q];
    vvc<bool> seen(H, vc<bool>(W,0));
    seen[P][Q]=1;
    pqg<pair<ll, pll>> q;
    vl dx = {1,-1,0,0}, dy = {0,0,1,-1};
    rep1(i,4){
        ll nh = P+dx[i], nw = Q + dy[i];
        if(nh>=0&&nh<H&&nw>=0&&nw<W&&!seen[nh][nw]) {
            q.push({A[nh][nw], {nh,nw}});
            seen[nh][nw]=1;
        }
    }
    while(q.size()){
        auto v = q.top();
        q.pop();
        if(v.first < ans/X + ((ans%X)?1:0)){
            ans += A[v.second.first][v.second.second];
            rep1(i,4){
                ll nh = v.second.first+dx[i], nw = v.second.second + dy[i];
                if(nh>=0&&nh<H&&nw>=0&&nw<W&&!seen[nh][nw]) {
                    q.push({A[nh][nw],{nh,nw}});
                    seen[nh][nw]=1;
                }
            }
        }
    }
    cout << ans << endl;
}