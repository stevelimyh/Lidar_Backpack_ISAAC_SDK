/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * @desc  Try to match a target string with a given string. A match is valid if it is possible to
 *        split the target string into subtrings such as all the subtrings are also subtrings of the
 *        other string, and in the same order. It trys to match using the minimum number of
 *        subtrings.
 * @param (string} text: The original text
 * @param {string} match: The matching string
 * @param {integer} infinity: A value considered to be infinity, if no match exist, it will return
 *                            this value.
 * @param {string} prefix: What to happen before a matching substring
 * @param {string} suffix: What to happen after a matching substring
 * @return the minimum number of splits required to get a valid match. If suffix and prefix are set
 *         it also returns the string containing the match.
 */
function TextFilter(text, match, infinity, prefix, suffix) {
  // Helper function to check if two characters are identical (a == A)
  let matching = function(a, b) {
    return a.toLowerCase() == b.toLowerCase();
  }
  // Create a dp array to compute the best match possible
  const text_size = text.length;
  const match_size = match.length;
  let dp = new Array(text_size + 1);
  for (let t = 0; t <= text_size; t++) {
    dp[t] = new Array(1 + match_size);
  }
  // First pass of the DP, we generate the best match from a given position
  for (let t = text_size; t >= 0; t--) {
    for (let m = match_size; m >= 0; m--) {
      if (m == match_size) {
        // We reach the end of the matching string, it takes 0 additional split
        dp[t][m] = 0;
      } else if (t == text_size) {
        // We reach the end of the text, there is not valid match anymore.
        dp[t][m] = infinity;
      } else {
        let ans = dp[t + 1][m];
        const max_size = Math.min(text_size - t, match_size - m);
        for (let i = 0; i < max_size && matching(match[m+i], text[t+i]); i++) {
          ans = Math.min(ans, 1 + dp[t+i+1][m+i+1]);
        }
        dp[t][m] = ans;
      }
    }
  }
  if (prefix === undefined || suffix == undefined) return dp[0][0];

  if (dp[0][0] == infinity) return {text: "", split: infinity};
  // Let's extract the best split.
  let ans = "";
  let m = 0;
  for (let t = 0; t < text_size;) {
    if (dp[t + 1][m] === dp[t][m]) {
      ans += text[t++];
    } else {
      let i = 0;
      while (t+i < text_size && m+i < match_size && matching(match[m+i], text[t+i])) {
        i++;
        if (dp[t + i][m + i] + 1 === dp[t][m]) break;
      }
      ans += prefix + text.substring(t, t+i) + suffix;
      t += i;
      m += i;
    }
  }
  return {text: ans, split: dp[0][0]};
};