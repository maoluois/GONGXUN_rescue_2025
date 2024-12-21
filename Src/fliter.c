//
// Created by Administrator on 24-9-23.
//
float mean_fliter(float *data, int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += data[i];
    }
    return sum / n;
}


