# Contributing to Materia

Thanks for helping. A few basics:

## License of contributions

Materia is licensed under the [Apache License 2.0](LICENSE). Documentation in
[`docs/`](docs/) is licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). By contributing, you
agree that your contribution is licensed under the same terms.

## Sign your commits (DCO)

Every commit must carry a `Signed-off-by:` line certifying the
[Developer Certificate of Origin](https://developercertificate.org/): that you
wrote the change, or otherwise have the right to submit it under the project's
license.

```sh
git commit -s
```

adds the line for you:

```
Signed-off-by: Your Name <you@example.com>
```

## Submodules

Haxeon and several native dependencies are Git submodules. When you change a
submodule, publish its commit before updating the submodule pointer in
Materia, so fresh recursive clones stay buildable.

## Where to start

- [README.md](README.md): building and running the reference editor.
- [docs/concepts/](docs/concepts/README.md): how Materia fits together.
- Each kit's README has its own build and test commands.
