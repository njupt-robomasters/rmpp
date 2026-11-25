// 在独立 Git 对象库和索引中模拟合并，保留工程分支、索引与工作区
const fs = require('node:fs');
const path = require('node:path');
const { spawnSync } = require('node:child_process');
const root = path.resolve(__dirname, '../../..');
const out = path.join(__dirname, '../../out', path.basename(__dirname));
const repo = path.join(out, 'review.git');
fs.mkdirSync(out, { recursive: true });

function git(args, isolated = false, input) {
    const result = spawnSync('git', ['-c', 'core.quotepath=false', '-c', 'core.safecrlf=false',
        ...(isolated ? [`--git-dir=${repo}`, `--work-tree=${root}`] : []), ...args], {
        cwd: root, input, encoding: 'utf8', maxBuffer: 32 * 1024 * 1024,
        env: { ...process.env, GIT_AUTHOR_NAME: 'Merge review', GIT_AUTHOR_EMAIL: 'review@local',
            GIT_COMMITTER_NAME: 'Merge review', GIT_COMMITTER_EMAIL: 'review@local' }
    });
    if (result.error || ![0, 1].includes(result.status)) throw result.error || Error(result.stderr);
    return { text: result.stdout.trim(), code: result.status };
}
const head = git(['rev-parse', 'HEAD']).text;
const upstream = git(['rev-parse', 'origin/master']).text;
const base = git(['merge-base', head, upstream]).text;
if (!fs.existsSync(repo)) git(['init', '--bare', repo]);
const objects = path.resolve(root, git(['rev-parse', '--git-path', 'objects']).text).replaceAll('\\', '/');
fs.writeFileSync(path.join(repo, 'objects/info/alternates'), objects + '\n');
const autocrlf = git(['config', '--get', 'core.autocrlf']).text;
if (autocrlf) git(['config', 'core.autocrlf', autocrlf], true);
git(['read-tree', head], true);
git(['add', '-u', '--', '.'], true);
const tree = git(['write-tree'], true).text;
const snapshot = git(['commit-tree', tree, '-p', head], true, 'Temporary tracked working tree snapshot\n').text;
const summary = { head, upstream, base, snapshot, results: {} };
for (const [label, local] of [['committed', head], ['working', snapshot]]) {
    const result = git(['merge-tree', '--write-tree', local, upstream], true);
    fs.writeFileSync(path.join(out, `${label}_merge.txt`), result.text + '\n');
    summary.results[label] = { exit_code: result.code, tree: result.text.split('\n')[0],
        conflicts: [...new Set(result.text.split('\n').filter(s => s.includes('\t')).map(s => s.split('\t')[1]))].sort() };
}
const lines = args => git(args).text.split('\n').filter(Boolean);
const changed = lines(['diff', '--name-only', 'HEAD']);
const upstreamChanged = new Set(lines(['diff', '--name-only', base, upstream]));
const upstreamPaths = new Set(lines(['ls-tree', '-r', '--name-only', upstream]));
summary.tracked_changes = changed;
summary.overlap = changed.filter(s => upstreamChanged.has(s));
summary.untracked_path_collisions = lines(['ls-files', '--others', '--exclude-standard']).filter(s => upstreamPaths.has(s));
fs.writeFileSync(path.join(out, 'summary.json'), JSON.stringify(summary, null, 2) + '\n');
console.log(JSON.stringify(summary, null, 2));
