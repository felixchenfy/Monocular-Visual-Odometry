

### vector

* speed:
almost same as array
https://stackoverflow.com/questions/8848575/fastest-way-to-reset-every-value-of-stdvectorint-to-0

* set all to zero
https://stackoverflow.com/questions/8848575/fastest-way-to-reset-every-value-of-stdvectorint-to-0
std::fill(v.begin(), v.end(), 0);//13.4s. But this is recommaneded stype.
memset(&v[0], 0, v.size() * sizeof v[0]); //0.125s

* resize and clear

* copy
` vector B(3,0)
vector A = B; // This is a memory copy. Later if change B, A won't change.
` use swap

### deque
deque is basically same as vector, but:  
supports two-ends constant-time insert and delete, while vector only supports one end.

