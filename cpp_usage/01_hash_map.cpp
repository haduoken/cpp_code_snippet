#include <hash_map>
#include <iostream>
#include <map>
using namespace std;
using __gnu_cxx::hash_map;
#include <unordered_map>

class Student {
 public:
  Student(string n, int a) : name_(n), age_(a) {}
  string name_;
  int age_;
};
struct hash_Student {
  size_t operator()(const class Student &A) const {
    //  return  hash<int>(classA.getvalue());
    return A.age_;
  }
};

// 2 define the equal function
struct equal_Student {
  bool operator()(const class Student &a1, const class Student &a2) const { return a1.age_ == a2.age_; }
};

struct less_Student {
  bool operator()(const class Student &a1, const class Student &a2) const { return a1.age_ < a2.age_; }
};

int main(int argc, char *argv[]) {
  hash_map<Student, string, hash_Student, equal_Student> hash_map;
  map<Student, string, less_Student> normal_map;

  Student foo("foo", 10);
  Student bar("bar", 20);
  Student test2("test2", 20);

  // C11之前的hash_map
  hash_map[foo] = "foo is 10";
  hash_map[bar] = "bar is 20";
  hash_map[test2] = "test hash conflict";

  Student test1("test1", 30);
  cout << "hash map is " << hash_map.bucket_count() << endl;

  cout << hash_map[bar] << endl;

  // map使用
  normal_map[foo] = "foo is 10";
  normal_map[bar] = "bar is 20";
  for (auto it = normal_map.begin(); it != normal_map.end(); it++) {
    cout << it->first.name_ << " value is " << it->second << endl;
  }

  auto it = normal_map.find(bar);
  cout << it->first.name_ << endl;

  // 官方版本的hash_map
  unordered_map<Student, string, hash_Student, equal_Student> organ_hash_map;
  organ_hash_map[foo] = "foo is 10";
  organ_hash_map[bar] = "bar is 20";
  organ_hash_map[test2] = "test hash conflict";
  cout << organ_hash_map[bar] << endl;
}
