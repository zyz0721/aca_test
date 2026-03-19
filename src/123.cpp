#include <vector>
using namespace std;

int search(vector<int>& nums, int target) {
    int left = 0;
    int right = nums.size() - 1;

    while (left <= right) {
        int mid = left + (right - left) / 2;

        if (nums[mid] == target) {
            return mid;
        }

        // 核心逻辑：判断哪一半是有序的
        // 技巧：如果 nums[left] <= nums[mid]，说明左半边 [left...mid] 是连续递增的
        // (比如 [4,5,6,7,0,1,2]，mid在7的位置，4<=7，左侧正常)
        if (nums[left] <= nums[mid]) {
            // 左半边有序，检查 target 是否在左半边的范围内
            if (nums[left] <= target && target < nums[mid]) {
                right = mid - 1; // target 在左边，缩圈
            } else {
                left = mid + 1;  // target 不在左边，去乱序的右边找
            }
        } 
        // 否则，右半边 [mid...right] 一定是有序的
        // (比如 [6,7,0,1,2,4,5]，mid在1的位置，6>1，说明左侧断裂，右侧正常)
        else {
            // 右半边有序，检查 target 是否在右半边的范围内
            if (nums[mid] < target && target <= nums[right]) {
                left = mid + 1; // target 在右边
            } else {
                right = mid - 1; // target 不在右边
            }
        }
    }

    return -1;
}